import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

type OccupancyLayer = "map" | "costmap" | "dynamic";
type TopicVisibility = {
  map: boolean;
  costmap: boolean;
  lidar: boolean;
  scan: boolean;
  path: boolean;
  robotModel: boolean;
};

type InteractionMode = "initialPose" | "goal" | null;

interface PointField {
  name: string;
  offset: number;
}

type TfEntry = {
  parent: string;
  translation: THREE.Vector3;
  rotation: THREE.Quaternion;
};

type PoseLike = {
  pose?: {
    position?: { x?: number; y?: number; z?: number };
  };
  header?: { frame_id?: string };
};
const MAP_TOPICS = ["/map", "/maps_manager_node/costmap/map"] as const;
const EASYNAV_MAP_TOPICS = ["/dynamic_map", "/maps_manager_node/simple/map", "/maps_manager_node/simple/dynamic_map"] as const;
const DYNAMIC_MAP_TOPICS = ["/maps_manager_node/costmap/dynamic_map"] as const;
const COSTMAP_TOPICS = ["/global_costmap/costmap"] as const;
const PATH_TOPICS = ["/received_global_plan", "/plan", "/plan_smoothed", "/planner_node/simple/path"] as const;
// Los topicos de pose se separan para priorizar la localizacion sobre la odometria.
const LOCALIZATION_POSE_TOPICS = ["/amcl_pose", "/localizer_node/costmap/pose"] as const;
const ODOMETRY_POSE_TOPICS = ["/odom", "/odometry/filtered"] as const;
const POSE_TOPICS = [...LOCALIZATION_POSE_TOPICS, ...ODOMETRY_POSE_TOPICS] as const;

// --- Utilidades ---
function quaternionToYaw(o: { x: number; y: number; z: number; w: number }): number {
  return Math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z));
}

function yawToQuaternion(yaw: number): { x: number; y: number; z: number; w: number } {
  const half = yaw / 2;
  return { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) };
}

function createRosNowStamp(): { sec: number; nanosec: number } {
  const nowMs = Date.now();
  const sec = Math.floor(nowMs / 1000);
  const nanosec = Math.floor((nowMs % 1000) * 1_000_000);
  return { sec, nanosec };
}

function parseNumberList(value: string | null | undefined): number[] {
  if (!value) {
    return [];
  }
  return value
    .trim()
    .split(/\s+/)
    .map((part) => Number(part))
    .filter((part) => Number.isFinite(part));
}

function parseUrdfOrigin(originElement: Element | null): { position: THREE.Vector3; rotation: THREE.Quaternion } {
  const xyzValues = parseNumberList(originElement?.getAttribute("xyz"));
  const rpyValues = parseNumberList(originElement?.getAttribute("rpy"));
  return {
    position: new THREE.Vector3(xyzValues[0] ?? 0, xyzValues[1] ?? 0, xyzValues[2] ?? 0),
    rotation: new THREE.Quaternion().setFromEuler(
      new THREE.Euler(rpyValues[0] ?? 0, rpyValues[1] ?? 0, rpyValues[2] ?? 0, "XYZ"),
    ),
  };
}

function getFirstChildElementByTagName(parent: Element, tagName: string): Element | null {
  return Array.from(parent.children).find((child) => child.tagName === tagName) ?? null;
}

function createRobotMaterial(color: number): THREE.MeshPhongMaterial {
  return new THREE.MeshPhongMaterial({ color, shininess: 24, flatShading: false });
}

function buildPrimitiveGeometry(geometryElement: Element): THREE.Mesh | null {
  const boxElement = getFirstChildElementByTagName(geometryElement, "box");
  if (boxElement) {
    const sizeValues = parseNumberList(boxElement.getAttribute("size"));
    return new THREE.Mesh(
      new THREE.BoxGeometry(sizeValues[0] ?? 0.1, sizeValues[1] ?? 0.1, sizeValues[2] ?? 0.1),
      createRobotMaterial(0x9da7b1),
    );
  }

  const cylinderElement = getFirstChildElementByTagName(geometryElement, "cylinder");
  if (cylinderElement) {
    const radius = Number(cylinderElement.getAttribute("radius") ?? 0.05);
    const length = Number(cylinderElement.getAttribute("length") ?? 0.1);
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 16), createRobotMaterial(0x6d7d8b));
    mesh.rotation.x = Math.PI / 2;
    return mesh;
  }

  const sphereElement = getFirstChildElementByTagName(geometryElement, "sphere");
  if (sphereElement) {
    const radius = Number(sphereElement.getAttribute("radius") ?? 0.05);
    return new THREE.Mesh(new THREE.SphereGeometry(radius, 16, 12), createRobotMaterial(0x9da7b1));
  }

  return null;
}

function buildRobotModelFromUrdf(urdf: string): THREE.Object3D | null {
  // Este parser construye intencionadamente solo desde primitivas de <collision>.
  // Asi evitamos dependencias de archivos mesh para que el panel funcione en distintos dispositivos.
  const parser = new DOMParser();
  const xml = parser.parseFromString(urdf, "text/xml");
  if (xml.querySelector("parsererror")) {
    return null;
  }

  const robotElement = xml.querySelector("robot");
  if (!robotElement) {
    return null;
  }

  const linkElements = Array.from(robotElement.getElementsByTagName("link"));
  const jointElements = Array.from(robotElement.getElementsByTagName("joint"));
  if (linkElements.length === 0) {
    return null;
  }

  const linkByName = new Map<string, Element>();
  for (const linkElement of linkElements) {
    const name = linkElement.getAttribute("name");
    if (name) {
      linkByName.set(name, linkElement);
    }
  }

  const parentByChild = new Map<string, { parent: string; origin: { position: THREE.Vector3; rotation: THREE.Quaternion } }>();
  const childNames = new Set<string>();
  for (const jointElement of jointElements) {
    const child = jointElement.querySelector("child")?.getAttribute("link") ?? "";
    const parent = jointElement.querySelector("parent")?.getAttribute("link") ?? "";
    if (!child || !parent) {
      continue;
    }

    parentByChild.set(child, {
      parent,
      origin: parseUrdfOrigin(jointElement.querySelector("origin")),
    });
    childNames.add(child);
  }

  const linkGroups = new Map<string, THREE.Group>();
  for (const [name, linkElement] of linkByName.entries()) {
    const linkGroup = new THREE.Group();
    linkGroup.name = name;

    const collisionElements = Array.from(linkElement.getElementsByTagName("collision"));
    for (const collisionElement of collisionElements) {
      const geometryElement = collisionElement.querySelector("geometry");
      if (!geometryElement) {
        continue;
      }

      const primitive = buildPrimitiveGeometry(geometryElement);
      if (!primitive) {
        continue;
      }

      const origin = parseUrdfOrigin(collisionElement.querySelector("origin"));
      const collisionGroup = new THREE.Group();
      collisionGroup.position.copy(origin.position);
      collisionGroup.quaternion.copy(origin.rotation);
      collisionGroup.add(primitive);
      linkGroup.add(collisionGroup);
    }

    linkGroups.set(name, linkGroup);
  }

  for (const [child, data] of parentByChild.entries()) {
    const parentGroup = linkGroups.get(data.parent);
    const childGroup = linkGroups.get(child);
    if (!parentGroup || !childGroup) {
      continue;
    }

    childGroup.position.copy(data.origin.position);
    childGroup.quaternion.copy(data.origin.rotation);
    parentGroup.add(childGroup);
  }

  const rootGroup = new THREE.Group();
  rootGroup.name = "robot_model";
  for (const [name, group] of linkGroups.entries()) {
    if (!childNames.has(name)) {
      rootGroup.add(group);
    }
  }

  return rootGroup.children.length > 0 ? rootGroup : null;
}

function extractPose(msg: any): { x: number; y: number; yaw: number } | null {
  const poseObj = msg?.pose?.pose ?? msg?.pose;
  const p = poseObj?.position;
  const q = poseObj?.orientation;
  if (
    typeof p?.x !== "number" ||
    typeof p?.y !== "number" ||
    typeof q?.x !== "number" ||
    typeof q?.y !== "number" ||
    typeof q?.z !== "number" ||
    typeof q?.w !== "number"
  ) {
    return null;
  }
  return { x: p.x, y: p.y, yaw: quaternionToYaw(q) };
}

function createPoseCovariance(): number[] {
  const covariance = new Array(36).fill(0);
  covariance[0] = 0.25;
  covariance[7] = 0.25;
  covariance[35] = 0.06853891909122467;
  return covariance;
}

function disposeObject3D(object: THREE.Object3D): void {
  object.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (mesh.geometry) {
      mesh.geometry.dispose();
    }
    const material = mesh.material;
    if (material) {
      if (Array.isArray(material)) {
        for (const m of material) {
          const tex = (m as THREE.MeshBasicMaterial).map;
          tex?.dispose();
          m.dispose();
        }
      } else {
        const tex = (material as THREE.MeshBasicMaterial).map;
        tex?.dispose();
        material.dispose();
      }
    }
  });
}

function asInt8(value: number): number {
  if (value > 127) {
    return value - 256;
  }
  return value;
}

function getPointFieldOffset(fields: PointField[] | undefined, name: string, fallback: number): number {
  if (!fields) {
    return fallback;
  }
  const found = fields.find((f) => f.name === name);
  return found?.offset ?? fallback;
}

function extractRobotDescription(msg: unknown): string | undefined {
  if (typeof msg === "string") {
    return msg;
  }
  if (typeof msg === "object" && msg != null) {
    const data = (msg as { data?: unknown }).data;
    if (typeof data === "string") {
      return data;
    }
  }
  return undefined;
}

function normalizeFrameId(frameId: string | undefined): string {
  if (!frameId) {
    return "";
  }
  return frameId.replace(/^\/+/, "");
}

function topicMatches(topic: string, candidates: readonly string[]): boolean {
  return candidates.indexOf(topic) !== -1;
}

function isEasyNavMapTopic(topic: string): boolean {
  return topicMatches(topic, EASYNAV_MAP_TOPICS);
}

function Visualization3DPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef(new THREE.Scene());
  const robotRootRef = useRef(new THREE.Group());
  const interactionPlaneRef = useRef<THREE.Mesh | null>(null);
  const previewArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const previewStartMarkerRef = useRef<THREE.Mesh | null>(null);
  const raycasterRef = useRef(new THREE.Raycaster());
  const pointerRef = useRef(new THREE.Vector2());
  const interactionDraftRef = useRef<{ start: THREE.Vector3; current: THREE.Vector3 } | null>(null);

  // Referencias para actualizaciones rapidas sin recrear objetos.
  const lidarPointsRef = useRef<THREE.Points | null>(null);
  const scanPointsRef = useRef<THREE.Points | null>(null);
  const lidarGeomRef = useRef(new THREE.BufferGeometry());
  const scanGeomRef = useRef(new THREE.BufferGeometry());
  const mapMeshRef = useRef<THREE.Mesh | null>(null);
  const costmapMeshRef = useRef<THREE.Mesh | null>(null);
  const pathLineRef = useRef<THREE.Line | null>(null);
  const robotModelRef = useRef<THREE.Object3D | null>(null);
  const goalMarkerRef = useRef<THREE.Mesh | null>(null);
  const goalArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const lastGoalPoseRef = useRef<{ x: number; y: number; yaw: number } | null>(null);
  const lastUrdfRef = useRef<string>("");
  const lastMapMsgRef = useRef<unknown>(null);
  const lastCostmapMsgRef = useRef<unknown>(null);
  const lastDynamicMapMsgRef = useRef<unknown>(null);
  const dynamicMapMeshRef = useRef<THREE.Mesh | null>(null);
  const lastLidarMsgRef = useRef<unknown>(null);
  const lastScanMsgRef = useRef<unknown>(null);
  const lastAmclPoseRef = useRef<unknown>(null);
  const lastGoalPoseMsgRef = useRef<unknown>(null);
  const lastTfMsgRef = useRef<unknown>(null);
  const lastTfStaticMsgRef = useRef<unknown>(null);
  const pendingInitialPoseRef = useRef<{ x: number; y: number; yaw: number; setAtMs: number } | null>(null);
  const hasLocalizationPoseRef = useRef<boolean>(false);
  const tfTreeRef = useRef<Map<string, TfEntry>>(new Map());
  const mapFrameRef = useRef<string>("map");
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);

  const [visibility, setVisibility] = useState<TopicVisibility>({
    map: true,
    costmap: true,
    lidar: true,
    scan: true,
    path: true,
    robotModel: true,
  });
  const [interactionMode, setInteractionMode] = useState<InteractionMode>(null);
  const [stackMode, setStackMode] = useState<"nav2" | "easynav" | null>(null);
  const showNav2Layers = stackMode !== "easynav";
  // Desplazamiento vertical de todo el robot respecto al plano del mapa z=0.
  // Subir si las ruedas atraviesan el mapa; bajar si el robot queda demasiado flotando.
  const robotLiftZ = 0.20;
  const topicRows = useMemo(
    () => [
      { key: "map", label: "Map", title: "Mapa base" },
      { key: "costmap", label: "Costmap", title: "Mapa de coste" },
      { key: "path", label: "Plan", title: "Ruta global" },
      { key: "lidar", label: "LiDAR", title: "Nube LiDAR" },
      { key: "scan", label: "Scan", title: "LaserScan" },
      { key: "robotModel", label: "Robot model", title: "Modelo robot" },
    ] as const,
    [],
  );
  const visibleTopicRows = showNav2Layers ? topicRows : topicRows.filter((row) => row.key === "map" || row.key === "lidar" || row.key === "scan" || row.key === "robotModel");

  const setVisible = (key: keyof TopicVisibility, checked: boolean): void => {
    setVisibility((prev) => ({ ...prev, [key]: checked }));
  };

  const setObjectVisible = (obj: THREE.Object3D | null, isVisible: boolean): void => {
    if (obj) {
      obj.visible = isVisible;
    }
  };

  const ensureGoalVisual = (): void => {
    if (!goalMarkerRef.current) {
      goalMarkerRef.current = new THREE.Mesh(
        new THREE.SphereGeometry(0.09, 16, 12),
        new THREE.MeshBasicMaterial({ color: 0x43d17c }),
      );
      sceneRef.current.add(goalMarkerRef.current);
    }

    if (!goalArrowRef.current) {
      goalArrowRef.current = new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0, 0.25),
        0.01,
        0x43d17c,
        0.2,
        0.12,
      );
      sceneRef.current.add(goalArrowRef.current);
    }
  };

  const updateGoalVisual = (x: number, y: number, yaw: number): void => {
    ensureGoalVisual();
    lastGoalPoseRef.current = { x, y, yaw };

    if (goalMarkerRef.current) {
      goalMarkerRef.current.position.set(x, y, 0.09);
      goalMarkerRef.current.visible = true;
    }

    if (goalArrowRef.current) {
      goalArrowRef.current.position.set(x, y, 0.2);
      goalArrowRef.current.setDirection(new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0).normalize());
      goalArrowRef.current.setLength(0.8, 0.22, 0.12);
      goalArrowRef.current.setColor(new THREE.Color(0x43d17c));
      goalArrowRef.current.visible = true;
    }
  };

  const clearPoseInteractionPreview = (): void => {
    interactionDraftRef.current = null;
    if (previewArrowRef.current) {
      sceneRef.current.remove(previewArrowRef.current);
      disposeObject3D(previewArrowRef.current);
      previewArrowRef.current = null;
    }
    if (previewStartMarkerRef.current) {
      sceneRef.current.remove(previewStartMarkerRef.current);
      disposeObject3D(previewStartMarkerRef.current);
      previewStartMarkerRef.current = null;
    }
  };

  const ensureInteractionHelpers = (mode: Exclude<InteractionMode, null>): void => {
    if (!previewStartMarkerRef.current) {
      previewStartMarkerRef.current = new THREE.Mesh(
        new THREE.SphereGeometry(0.08, 16, 12),
        new THREE.MeshBasicMaterial({ color: mode === "initialPose" ? 0x4da3ff : 0x43d17c }),
      );
      sceneRef.current.add(previewStartMarkerRef.current);
    }

    if (!previewArrowRef.current) {
      previewArrowRef.current = new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0, 0.3),
        0.01,
        mode === "initialPose" ? 0x4da3ff : 0x43d17c,
        0.18,
        0.12,
      );
      sceneRef.current.add(previewArrowRef.current);
    }
  };

  const updateInteractionPreview = (start: THREE.Vector3, current: THREE.Vector3, mode: Exclude<InteractionMode, null>): void => {
    ensureInteractionHelpers(mode);

    if (previewStartMarkerRef.current) {
      previewStartMarkerRef.current.position.set(start.x, start.y, 0.08);
      (previewStartMarkerRef.current.material as THREE.MeshBasicMaterial).color.set(
        mode === "initialPose" ? 0x4da3ff : 0x43d17c,
      );
    }

    if (previewArrowRef.current) {
      const direction = new THREE.Vector3(current.x - start.x, current.y - start.y, 0);
      const length = Math.max(direction.length(), 0.01);
      direction.normalize();
      previewArrowRef.current.position.set(start.x, start.y, 0.2);
      previewArrowRef.current.setDirection(direction);
      previewArrowRef.current.setLength(length, length * 0.25, length * 0.18);
      previewArrowRef.current.setColor(new THREE.Color(mode === "initialPose" ? 0x4da3ff : 0x43d17c));
    }
  };

  const publishInitialPose = (x: number, y: number, yaw: number): void => {
    const topic = "/initialpose";
    const datatype = "geometry_msgs/PoseWithCovarianceStamped";
    context.advertise?.(topic, datatype);

    context.publish?.(topic, {
      header: { stamp: createRosNowStamp(), frame_id: "map" },
      pose: {
        pose: {
          position: { x, y, z: 0 },
          orientation: yawToQuaternion(yaw),
        },
        covariance: createPoseCovariance(),
      },
    });

    // Aplica la estimacion inmediatamente en la escena local para que robot/LiDAR no parezcan congelados
    // mientras se espera a que el stack de localizacion publique el topico de pose actualizado.
    robotRootRef.current.position.set(x, y, robotLiftZ);
    robotRootRef.current.rotation.z = yaw;
    pendingInitialPoseRef.current = { x, y, yaw, setAtMs: Date.now() };
  };

  const publishGoalPose = (x: number, y: number, yaw: number): void => {
    const topic = "/goal_pose";
    const datatype = "geometry_msgs/PoseStamped";
    context.advertise?.(topic, datatype);

    context.publish?.(topic, {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
      pose: {
        position: { x, y, z: 0 },
        orientation: yawToQuaternion(yaw),
      },
    });

    updateGoalVisual(x, y, yaw);
  };

  const getIntersectionPoint = (event: PointerEvent): THREE.Vector3 | null => {
    const camera = cameraRef.current;
    const renderer = rendererRef.current;
    const plane = interactionPlaneRef.current;
    if (!camera || !renderer || !plane) {
      return null;
    }

    const rect = renderer.domElement.getBoundingClientRect();
    pointerRef.current.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -(((event.clientY - rect.top) / rect.height) * 2 - 1),
    );
    raycasterRef.current.setFromCamera(pointerRef.current, camera);
    const hit = raycasterRef.current.intersectObject(plane, false)[0];
    return hit ? hit.point.clone() : null;
  };

  const finishPoseInteraction = (): void => {
    const draft = interactionDraftRef.current;
    if (!draft || !interactionMode) {
      return;
    }

    const dx = draft.current.x - draft.start.x;
    const dy = draft.current.y - draft.start.y;
    const yaw = Math.hypot(dx, dy) > 0.01 ? Math.atan2(dy, dx) : 0;

    if (interactionMode === "initialPose") {
      publishInitialPose(draft.start.x, draft.start.y, yaw);
    } else {
      publishGoalPose(draft.start.x, draft.start.y, yaw);
    }

    clearPoseInteractionPreview();
    setInteractionMode(null);
  };

  // --- 1. Inicializacion de escena ---
  useEffect(() => {
    if (!containerRef.current) return;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    rendererRef.current = renderer;
    renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0xf2f2f2);
    containerRef.current.appendChild(renderer.domElement);

    const aspect = containerRef.current.clientWidth / Math.max(containerRef.current.clientHeight, 1);
    const camera = new THREE.PerspectiveCamera(60, aspect, 0.05, 2000);
    cameraRef.current = camera;
    camera.position.set(-5, -5, 10);
    camera.up.set(0, 0, 1);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controlsRef.current = controls;

    // Iluminacion minima para el URDF y anade la raiz del robot a la escena.
    sceneRef.current.add(new THREE.AmbientLight(0xffffff, 0.85));
    sceneRef.current.add(robotRootRef.current);
    interactionPlaneRef.current = new THREE.Mesh(
      new THREE.PlaneGeometry(2000, 2000),
      new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthWrite: false, depthTest: false }),
    );
    interactionPlaneRef.current.position.z = 0.22;
    sceneRef.current.add(interactionPlaneRef.current);

    const onResize = (): void => {
      if (!containerRef.current || !rendererRef.current || !cameraRef.current) {
        return;
      }
      const width = containerRef.current.clientWidth;
      const height = Math.max(containerRef.current.clientHeight, 1);
      rendererRef.current.setSize(width, height);
      cameraRef.current.aspect = width / height;
      cameraRef.current.updateProjectionMatrix();
    };
    const resizeObserver = new ResizeObserver(onResize);
    resizeObserver.observe(containerRef.current);
    window.addEventListener("resize", onResize);
    onResize();

    let animationFrame = 0;
    const animate = (): void => {
      animationFrame = requestAnimationFrame(animate);
      controls.update();
      renderer.render(sceneRef.current, camera);
    };
    animate();

    return () => {
      cancelAnimationFrame(animationFrame);
      resizeObserver.disconnect();
      window.removeEventListener("resize", onResize);
      for (const obj of [mapMeshRef.current, costmapMeshRef.current, dynamicMapMeshRef.current, pathLineRef.current, lidarPointsRef.current, scanPointsRef.current]) {
        if (obj) {
          sceneRef.current.remove(obj);
          disposeObject3D(obj);
        }
      }
      clearPoseInteractionPreview();
      if (interactionPlaneRef.current) {
        sceneRef.current.remove(interactionPlaneRef.current);
        disposeObject3D(interactionPlaneRef.current);
        interactionPlaneRef.current = null;
      }
      if (goalArrowRef.current) {
        sceneRef.current.remove(goalArrowRef.current);
        disposeObject3D(goalArrowRef.current);
        goalArrowRef.current = null;
      }
      if (goalMarkerRef.current) {
        sceneRef.current.remove(goalMarkerRef.current);
        disposeObject3D(goalMarkerRef.current);
        goalMarkerRef.current = null;
      }
      if (robotModelRef.current) {
        robotRootRef.current.remove(robotModelRef.current);
        disposeObject3D(robotModelRef.current);
      }
      renderer.dispose();
      controls.dispose();
    };
  }, []);

  // --- 2. Procesado LiDAR ---
  const updateLidar = (msg: any): void => {
    if (!msg?.data) {
      return;
    }
    // Filtros LiDAR para ajustar rapidamente legibilidad frente a cobertura:
    // - maxRange: recorta puntos lejanos
    // - minZ: oculta puntos por debajo del suelo/plano del mapa
    const maxRange = 2.0;
    const minRange = 0.05;
    const minZ = -0.22;
    const step = msg.point_step ?? 16;
    const buf = msg.data instanceof Uint8Array ? msg.data : new Uint8Array(msg.data);
    const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
    const fields = msg.fields as PointField[] | undefined;
    const xOffset = getPointFieldOffset(fields, "x", 0);
    const yOffset = getPointFieldOffset(fields, "y", 4);
    const zOffset = getPointFieldOffset(fields, "z", 8);
    const points: number[] = [];

    for (let i = 0; i < Math.floor(buf.length / step); i++) {
      const off = i * step;
      if (off + Math.max(xOffset, yOffset, zOffset) + 4 > buf.length) {
        break;
      }
      const x = dv.getFloat32(off + xOffset, true);
      const y = dv.getFloat32(off + yOffset, true);
      const z = dv.getFloat32(off + zOffset, true);
      const r = Math.hypot(x, y);

      if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z) && z >= minZ && r >= minRange && r <= maxRange) {
        points.push(x, y, z);
      }
    }

    lidarGeomRef.current.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));

    if (!lidarPointsRef.current) {
      const mat = new THREE.PointsMaterial({ color: 0xff4545, size: 0.03, sizeAttenuation: true });
      lidarPointsRef.current = new THREE.Points(lidarGeomRef.current, mat);
      robotRootRef.current.add(lidarPointsRef.current);
    }

    const posAttr = lidarGeomRef.current.getAttribute("position") as THREE.BufferAttribute | undefined;
    if (posAttr != undefined) {
      posAttr.needsUpdate = true;
    }
  };

  const updateScan = (msg: any): void => {
    const ranges = msg?.ranges as ArrayLike<number> | undefined;
    if (!ranges || typeof ranges.length !== "number") {
      return;
    }
    const angleMin = typeof msg.angle_min === "number" ? msg.angle_min : 0;
    const angleInc = typeof msg.angle_increment === "number" ? msg.angle_increment : 0;
    const rangeMin = typeof msg.range_min === "number" ? msg.range_min : 0.05;
    const rangeMax = Math.min(typeof msg.range_max === "number" ? msg.range_max : 12.0, 12.0);
    const points: number[] = [];

    for (let i = 0; i < ranges.length; i++) {
      const range = ranges[i] as number;
      if (!Number.isFinite(range) || range < rangeMin || range > rangeMax) {
        continue;
      }
      const angle = angleMin + i * angleInc;
      points.push(range * Math.cos(angle), range * Math.sin(angle), 0.02);
    }

    scanGeomRef.current.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));
    if (!scanPointsRef.current) {
      const mat =
       new THREE.PointsMaterial({ color: 0x9b59ff, size: 0.075, sizeAttenuation: true });
      scanPointsRef.current = new THREE.Points(scanGeomRef.current, mat);
      robotRootRef.current.add(scanPointsRef.current);
    }
    const posAttr = scanGeomRef.current.getAttribute("position") as THREE.BufferAttribute | undefined;
    if (posAttr != undefined) {
      posAttr.needsUpdate = true;
    }
  };

  // --- 3. Capas de ocupacion ---
  const updateOccupancy = (msg: any, layer: OccupancyLayer): void => {
    if (!msg?.info || !msg?.data) {
      return;
    }
    const { width, height, resolution, origin } = msg.info;
    const frameId = normalizeFrameId(msg?.header?.frame_id);
    if (frameId) {
      mapFrameRef.current = frameId;
    }
    const data = msg.data as ArrayLike<number>;
    if (!width || !height || !resolution) {
      return;
    }
    const pixels = new Uint8Array(width * height * 4);

    for (let i = 0; i < data.length; i++) {
      const raw = data[i] ?? -1;
      const v = asInt8(raw);
      const idx = i * 4;
      if (layer === "map") {
        if (v < 0) {
          pixels[idx + 3] = 0;
        } else if (v === 0) {
          pixels[idx] = 255;
          pixels[idx + 1] = 255;
          pixels[idx + 2] = 255;
          pixels[idx + 3] = 255;
        } else if (v >= 100) {
          pixels[idx] = 0;
          pixels[idx + 1] = 0;
          pixels[idx + 2] = 0;
          pixels[idx + 3] = 255;
        } else {
          const shade = 220 - Math.round(v * 1.6);
          pixels[idx] = shade;
          pixels[idx + 1] = shade;
          pixels[idx + 2] = shade;
          pixels[idx + 3] = 220;
        }
      } else if(layer === "costmap") {
        if (v < 0) {
          pixels[idx + 3] = 0;
        } else if (v === 0) {
          // Si el mapa base esta oculto, aclara las celdas libres para evitar una vista demasiado oscura.
          if (visibility.map) {
            // Mantenerlo visible por encima del mapa estatico, similar al overlay de costmap en RViz.
            pixels[idx] = 165;
            pixels[idx + 1] = 205;
            pixels[idx + 2] = 255;
            pixels[idx + 3] = 90;
          } else {
            pixels[idx] = 246;
            pixels[idx + 1] = 248;
            pixels[idx + 2] = 252;
            pixels[idx + 3] = 235;
          }
        } else if (v >= 254) {
          pixels[idx] = 255;
          pixels[idx + 1] = 32;
          pixels[idx + 2] = 32;
          pixels[idx + 3] = 235;
        } else if (v >= 253) {
          pixels[idx] = 255;
          pixels[idx + 1] = 235;
          pixels[idx + 2] = 60;
          pixels[idx + 3] = 230;
        } else if (v >= 120) {
          pixels[idx] = 255;
          pixels[idx + 1] = 165;
          pixels[idx + 2] = 45;
          pixels[idx + 3] = 205;
        } else if (v >= 100) {
          pixels[idx] = 255;
          pixels[idx + 1] = 120;
          pixels[idx + 2] = 40;
          pixels[idx + 3] = 215;
        } else if (v >= 60) {
          pixels[idx] = 255;
          pixels[idx + 1] = 205;
          pixels[idx + 2] = 65;
          pixels[idx + 3] = 195;
        } else {
          // Banda de inflacion baja en amarillo para emular el borde de obstaculos tipo RViz.
          pixels[idx] = 255;
          pixels[idx + 1] = 235;
          pixels[idx + 2] = 80;
          pixels[idx + 3] = 175;
        }
      } else if (layer === "dynamic") {
        if (v === 0) {
          // libre: transparente, deja ver el mapa base
          pixels[idx + 3] = 0;
        } else if (v >= 100) {
          // ocupado: negro
          pixels[idx] = 0; pixels[idx + 1] = 0; pixels[idx + 2] = 0; pixels[idx + 3] = 255;
        } else if (v < 0) {
          // inflación: valores negativos, cuanto más negativo más cerca del obstáculo
          const intensity = Math.min(Math.abs(v) / 70, 1); // -70 = máx inflación
          pixels[idx] = 255;
          pixels[idx + 1] = Math.round(220 * (1 - intensity));
          pixels[idx + 2] = 0;
          pixels[idx + 3] = Math.round(120 + intensity * 120);
        } else {
          pixels[idx + 3] = 0;
        }
      }
  }

    const tex = new THREE.DataTexture(pixels, width, height, THREE.RGBAFormat);
    tex.flipY = false;
    tex.magFilter = THREE.NearestFilter;
    tex.minFilter = THREE.NearestFilter;
    tex.needsUpdate = true;

    const mesh = layer === "map" ? mapMeshRef : layer === "costmap" ? costmapMeshRef : dynamicMapMeshRef;
    if (mesh.current) {
      sceneRef.current.remove(mesh.current);
      disposeObject3D(mesh.current);
    }

    const geometry = new THREE.PlaneGeometry(width * resolution, height * resolution);
    const material = new THREE.MeshBasicMaterial({
      map: tex,
      transparent: true,
      depthWrite: true,
      depthTest: true,
      alphaTest: 0.01,
      side: THREE.DoubleSide,
    });
    mesh.current = new THREE.Mesh(geometry, material);
    mesh.current.position.set(
      origin.position.x + (width * resolution) / 2,
      origin.position.y + (height * resolution) / 2,
      layer === "map" ? 0 : 0.02,
    );
    mesh.current.renderOrder = layer === "map" ? 1 : layer === "costmap" ? 2 : 3;
    mesh.current.visible = layer === "map" 
      ? visibility.map 
      : layer === "costmap" 
        ? showNav2Layers && visibility.costmap 
        : visibility.map;  // dynamic sigue la visibilidad del mapa base
    sceneRef.current.add(mesh.current);
  };

  const updateTfTree = (msg: any): void => {
    const transforms = msg?.transforms as
      | Array<{
          header?: { frame_id?: string };
          child_frame_id?: string;
          transform?: {
            translation?: { x?: number; y?: number; z?: number };
            rotation?: { x?: number; y?: number; z?: number; w?: number };
          };
        }>
      | undefined;
    if (!Array.isArray(transforms)) {
      return;
    }

    for (const tf of transforms) {
      const parent = normalizeFrameId(tf?.header?.frame_id);
      const child = normalizeFrameId(tf?.child_frame_id);
      if (!parent || !child) {
        continue;
      }

      const t = tf?.transform?.translation;
      const r = tf?.transform?.rotation;
      const tx = t?.x;
      const ty = t?.y;
      const tz = t?.z;
      const qx = r?.x;
      const qy = r?.y;
      const qz = r?.z;
      const qw = r?.w;
      if (
        typeof tx !== "number" ||
        typeof ty !== "number" ||
        typeof tz !== "number" ||
        typeof qx !== "number" ||
        typeof qy !== "number" ||
        typeof qz !== "number" ||
        typeof qw !== "number"
      ) {
        continue;
      }

      tfTreeRef.current.set(child, {
        parent,
        translation: new THREE.Vector3(tx, ty, tz),
        rotation: new THREE.Quaternion(qx, qy, qz, qw),
      });
    }
  };

  const transformPointToFrame = (point: THREE.Vector3, sourceFrameRaw: string, targetFrameRaw: string): THREE.Vector3 | null => {
    const sourceFrame = normalizeFrameId(sourceFrameRaw);
    const targetFrame = normalizeFrameId(targetFrameRaw);
    if (!sourceFrame || !targetFrame) {
      return null;
    }
    if (sourceFrame === targetFrame) {
      return point.clone();
    }

    const transformed = point.clone();
    let current = sourceFrame;
    const visited = new Set<string>();

    for (let depth = 0; depth < 100; depth++) {
      if (current === targetFrame) {
        return transformed;
      }
      if (visited.has(current)) {
        return null;
      }
      visited.add(current);

      const tf = tfTreeRef.current.get(current);
      if (!tf) {
        return null;
      }
      transformed.applyQuaternion(tf.rotation);
      transformed.add(tf.translation);
      current = tf.parent;
    }

    return null;
  };

  const updateGlobalPlan = (msg: any): void => {
    const poses = msg?.poses as PoseLike[] | undefined;
    if (!Array.isArray(poses) || poses.length === 0) {
      return;
    }

    const pathFrame = normalizeFrameId(msg?.header?.frame_id) || mapFrameRef.current;
    const targetFrame = mapFrameRef.current;

    const points: number[] = [];
    for (const pose of poses) {
      const position = pose?.pose?.position;
      if (!position) {
        continue;
      }
      const x = position.x;
      const y = position.y;
      if (typeof x !== "number" || typeof y !== "number") {
        continue;
      }
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        continue;
      }

      const poseFrame = normalizeFrameId(pose?.header?.frame_id) || pathFrame;
      const worldPoint = transformPointToFrame(new THREE.Vector3(x, y, 0), poseFrame, targetFrame);
      if (!worldPoint) {
        // Usa coordenadas directas como alternativa si la cadena TF aun no esta disponible.
        points.push(x, y, 0.08);
        continue;
      }
      points.push(worldPoint.x, worldPoint.y, 0.08);
    }

    if (points.length < 6) {
      return;
    }

    if (pathLineRef.current) {
      sceneRef.current.remove(pathLineRef.current);
      disposeObject3D(pathLineRef.current);
      pathLineRef.current = null;
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));
    const material = new THREE.LineBasicMaterial({
      color: 0x00d1ff,
      transparent: true,
      opacity: 0.95,
      depthTest: false,
      depthWrite: false,
    });
    pathLineRef.current = new THREE.Line(geometry, material);
    pathLineRef.current.renderOrder = 10;
    pathLineRef.current.visible = visibility.path;
    sceneRef.current.add(pathLineRef.current);
  };

  const applyRobotModel = (urdf: string, force = false): void => {
    if (!urdf || (!force && urdf === lastUrdfRef.current)) {
      return;
    }
    lastUrdfRef.current = urdf;
    try {
      const model = buildRobotModelFromUrdf(urdf);
      if (!model) {
        throw new Error("Unable to build robot model from URDF");
      }

      model.name = "robot_model";
      model.traverse((obj: THREE.Object3D) => {
        const mesh = obj as THREE.Mesh;
        if (mesh.isMesh) {
          mesh.castShadow = false;
          mesh.receiveShadow = false;
        }
      });

      model.visible = visibility.robotModel;
      if (robotModelRef.current) {
        robotRootRef.current.remove(robotModelRef.current);
        disposeObject3D(robotModelRef.current);
      }
      robotModelRef.current = model;
      robotRootRef.current.add(model);
    } catch {
      // Mantiene estable el panel aunque falle el parseo del URDF; sin geometria de relleno.
    }
  };

  const updateRobotModel = (msg: unknown): void => {
    const urdf = extractRobotDescription(msg);
    if (!urdf) {
      return;
    }

    applyRobotModel(urdf);
  };

  // --- 4. Suscripciones ---
  useLayoutEffect(() => {
    // Ingestion por frame desde currentFrame de Foxglove.


    context.onRender = (renderState, done) => {
      // Busca el dynamic_map en allFrames si no llega por currentFrame
      const allTopicFrames = [
        ...(renderState.currentFrame ?? []),
      ];

      // Para topics que solo publican una vez, buscar también en allFrames
      const lastFramesByTopic = new Map<string, unknown>();
      for (const msgEvent of renderState.allFrames ?? []) {
        lastFramesByTopic.set(msgEvent.topic, msgEvent.message);
      }

      // Procesar dynamic_map desde allFrames si está disponible
      const dynamicMapMsg = lastFramesByTopic.get("/maps_manager_node/costmap/dynamic_map");
      if (dynamicMapMsg && dynamicMapMsg !== lastDynamicMapMsgRef.current) {
        lastDynamicMapMsgRef.current = dynamicMapMsg;
        updateOccupancy(dynamicMapMsg, "dynamic");
      }

      for (const msgEvent of allTopicFrames) {
        console.log("topic recibido:", msgEvent.topic);
        const m = msgEvent.message as any;
        if (msgEvent.topic === "/rslidar_points" && visibility.lidar) {
          lastLidarMsgRef.current = m;
          updateLidar(m);
        }
        if (msgEvent.topic === "/scan" && visibility.scan) {
          lastScanMsgRef.current = m;
          updateScan(m);
        }
        if (topicMatches(msgEvent.topic, MAP_TOPICS) && visibility.map && m !== lastMapMsgRef.current) {
          lastMapMsgRef.current = m;
          setStackMode("nav2");
          updateOccupancy(m, "map");
          continue;
        }
        if (isEasyNavMapTopic(msgEvent.topic) && visibility.map && m !== lastMapMsgRef.current) {
          lastMapMsgRef.current = m;
          setStackMode("easynav");
          updateOccupancy(m, "map");
          continue;
        }
        if (topicMatches(msgEvent.topic, COSTMAP_TOPICS) && showNav2Layers && visibility.costmap && m !== lastCostmapMsgRef.current) {
          lastCostmapMsgRef.current = m;
          setStackMode("nav2");
          updateOccupancy(m, "costmap");
        }
        if (topicMatches(msgEvent.topic, DYNAMIC_MAP_TOPICS)) {
          const data = m?.data as ArrayLike<number> | undefined;
          if (data) {
            const counts: Record<number, number> = {};
            for (let i = 0; i < Math.min(data.length, 10000); i++) {
              const v = asInt8((data[i] as number) ?? -1);
              counts[v] = (counts[v] ?? 0) + 1;
            }
            console.log("dynamic_map RECIBIDO, valores:", counts);
          } else {
            console.log("dynamic_map RECIBIDO pero sin data, msg:", m);
          }
          lastDynamicMapMsgRef.current = m;
          updateOccupancy(m, "dynamic");
        }

        if (topicMatches(msgEvent.topic, PATH_TOPICS) && showNav2Layers && visibility.path) {
          updateGlobalPlan(m);
        }
        if (topicMatches(msgEvent.topic, POSE_TOPICS)) {
          lastAmclPoseRef.current = m;
          const pose = extractPose(m);
          if (!pose) {
            continue;
          }
          const isLocalizationTopic = topicMatches(msgEvent.topic, LOCALIZATION_POSE_TOPICS);
          const isOdometryTopic = topicMatches(msgEvent.topic, ODOMETRY_POSE_TOPICS);

          // Mantener la localizacion como fuente autoritativa en cuanto este disponible.
          if (isLocalizationTopic) {
            hasLocalizationPoseRef.current = true;
          }

          // Evita que la odometria sobrescriba una estimacion 2D o la salida de AMCL/localizador.
          if (isOdometryTopic && (pendingInitialPoseRef.current != null || hasLocalizationPoseRef.current)) {
            continue;
          }

          const pending = pendingInitialPoseRef.current;
          if (pending && isLocalizationTopic) {
            const dist = Math.hypot(pose.x - pending.x, pose.y - pending.y);
            const yawDiff = Math.abs(Math.atan2(Math.sin(pose.yaw - pending.yaw), Math.cos(pose.yaw - pending.yaw)));

            // Ignora ecos de localizacion desactualizados hasta que coincidan con la pose recien estimada.
            if (dist > 0.4 || yawDiff > 0.8) {
              continue;
            }
            pendingInitialPoseRef.current = null;
          }

          robotRootRef.current.position.set(pose.x, pose.y, robotLiftZ);
          robotRootRef.current.rotation.z = pose.yaw;
        }
        if (msgEvent.topic === "/robot_description") {
          updateRobotModel(m);
        }
        if (msgEvent.topic === "/tf" && m !== lastTfMsgRef.current) {
          lastTfMsgRef.current = m;
          updateTfTree(m);
        }
        if (msgEvent.topic === "/tf_static" && m !== lastTfStaticMsgRef.current) {
          lastTfStaticMsgRef.current = m;
          updateTfTree(m);
        }
        if (msgEvent.topic === "/goal_pose" && m !== lastGoalPoseMsgRef.current) {
          lastGoalPoseMsgRef.current = m;
          const p = m.pose.position;
          const q = m.pose.orientation;
          const yaw = quaternionToYaw(q);
          updateGoalVisual(p.x, p.y, yaw);
        }
      }
      done();
    };
    context.watch("currentFrame");
    context.subscribe([
      { topic: "/rslidar_points" },
      { topic: "/scan" },
      ...MAP_TOPICS.map((topic) => ({ topic })),
      ...DYNAMIC_MAP_TOPICS.map((topic) => ({ topic })),
      ...EASYNAV_MAP_TOPICS.map((topic) => ({ topic })),
      ...(showNav2Layers ? COSTMAP_TOPICS.map((topic) => ({ topic })) : []),
      ...(showNav2Layers ? PATH_TOPICS.map((topic) => ({ topic })) : []),
      ...POSE_TOPICS.map((topic) => ({ topic })),
      { topic: "/robot_description" },
      { topic: "/tf" },
      { topic: "/tf_static" },
      { topic: "/goal_pose" },
    ]);
  }, [context, visibility, showNav2Layers]);

  useEffect(() => {
    setObjectVisible(mapMeshRef.current, visibility.map);
    setObjectVisible(costmapMeshRef.current, showNav2Layers && visibility.costmap);
    setObjectVisible(pathLineRef.current, showNav2Layers && visibility.path);
    setObjectVisible(lidarPointsRef.current, visibility.lidar);
    setObjectVisible(scanPointsRef.current, visibility.scan);
    setObjectVisible(robotModelRef.current, visibility.robotModel);
  }, [visibility, showNav2Layers]);

  useEffect(() => {
    if (controlsRef.current) {
      controlsRef.current.enabled = interactionMode === null;
    }

    const renderer = rendererRef.current;
    const canvas = renderer?.domElement;
    if (!canvas || interactionMode === null) {
      clearPoseInteractionPreview();
      return;
    }

    let isDragging = false;

    const onPointerDown = (event: PointerEvent): void => {
      event.preventDefault();
      const point = getIntersectionPoint(event);
      if (!point) {
        return;
      }

      isDragging = true;
      interactionDraftRef.current = { start: point.clone(), current: point.clone() };
      updateInteractionPreview(point, point, interactionMode);
    };

    const onPointerMove = (event: PointerEvent): void => {
      if (!isDragging || !interactionDraftRef.current) {
        return;
      }
      const point = getIntersectionPoint(event);
      if (!point) {
        return;
      }

      interactionDraftRef.current.current = point.clone();
      updateInteractionPreview(interactionDraftRef.current.start, point, interactionMode);
    };

    const onPointerUp = (event: PointerEvent): void => {
      if (!isDragging || !interactionDraftRef.current) {
        return;
      }
      event.preventDefault();
      const point = getIntersectionPoint(event);
      if (point) {
        interactionDraftRef.current.current = point.clone();
        updateInteractionPreview(interactionDraftRef.current.start, point, interactionMode);
      }

      finishPoseInteraction();
      isDragging = false;
    };

    const onContextMenu = (event: MouseEvent): void => {
      event.preventDefault();
    };

    canvas.addEventListener("pointerdown", onPointerDown);
    window.addEventListener("pointermove", onPointerMove);
    window.addEventListener("pointerup", onPointerUp);
    canvas.addEventListener("contextmenu", onContextMenu);

    return () => {
      canvas.removeEventListener("pointerdown", onPointerDown);
      window.removeEventListener("pointermove", onPointerMove);
      window.removeEventListener("pointerup", onPointerUp);
      canvas.removeEventListener("contextmenu", onContextMenu);
    };
  }, [interactionMode]);

  return (
    <div style={{ width: "100%", height: "100%", position: "relative", background: "#f2f2f2" }}>
      <div ref={containerRef} style={{ width: "100%", height: "100%" }} />
      <div
        style={{
          position: "absolute",
          top: 10,
          left: 10,
          background: "rgba(20,20,20,0.8)",
          color: "#f2f2f2",
          padding: "8px 10px",
          border: "1px solid #4b4b4b",
          borderRadius: 8,
          fontSize: 12,
          lineHeight: 1.5,
        }}
      >
        <div style={{ fontWeight: 700, marginBottom: 4 }}>Capas visibles</div>
        {visibleTopicRows.map((row) => (
          <label key={row.key} style={{ display: "block", cursor: "pointer" }}>
            <input
              type="checkbox"
              checked={visibility[row.key]}
              onChange={(e) => setVisible(row.key, e.target.checked)}
              style={{ marginRight: 6 }}
            />
            {row.title}
          </label>
        ))}
      </div>
      <div
        style={{
          position: "absolute",
          top: 10,
          right: 10,
          background: "rgba(20,20,20,0.8)",
          color: "#f2f2f2",
          padding: "8px 10px",
          border: "1px solid #4b4b4b",
          borderRadius: 8,
          fontSize: 12,
          lineHeight: 1.5,
          minWidth: 190,
          maxWidth: 340,
        }}
      >
        <div style={{ fontWeight: 700, marginBottom: 6 }}>Nav2 Tools</div>
        <button
          style={{
            width: "100%",
            marginBottom: 6,
            padding: "6px 8px",
            background: interactionMode === "initialPose" ? "#1f5cff" : "#555",
            color: "white",
            border: "none",
            borderRadius: 6,
            cursor: "pointer",
          }}
          onClick={() => {
            setInteractionMode((prev) => (prev === "initialPose" ? null : "initialPose"));
            clearPoseInteractionPreview();
          }}
        >
          {interactionMode === "initialPose" ? "Cancelar Pose Estimate" : "2D Pose Estimate"}
        </button>
        <button
          style={{
            width: "100%",
            marginBottom: 6,
            padding: "6px 8px",
            background: interactionMode === "goal" ? "#1f5cff" : "#555",
            color: "white",
            border: "none",
            borderRadius: 6,
            cursor: "pointer",
          }}
          onClick={() => {
            setInteractionMode((prev) => (prev === "goal" ? null : "goal"));
            clearPoseInteractionPreview();
          }}
        >
          {interactionMode === "goal" ? "Cancelar Goal" : "2D Goal Pose"}
        </button>
        <div style={{ color: "#cfcfcf" }}>
          Modo: {interactionMode === null ? "navegación" : interactionMode === "initialPose" ? "pose inicial" : "objetivo"}
        </div>
        <div style={{ color: "#9fa8da", marginTop: 4 }}>Click y arrastra sobre el mapa para orientar.</div>
      </div>
    </div>
  );
}

export function initVisualization3DPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<Visualization3DPanel context={context} />);
  return () => root.unmount();
}
