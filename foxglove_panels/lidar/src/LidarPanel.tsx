import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

interface PointField {
  name: string;
  offset: number;
  datatype: number;
  count: number;
}

interface RobotPose {
  x: number;
  y: number;
  yaw: number;
}

interface PoseCandidate {
  pose: RobotPose;
  priority: number;
}

const MAP_TOPICS = ["/map", "/maps_manager_node/costmap/map"] as const;
const COSTMAP_TOPICS = ["/global_costmap/costmap", "/maps_manager_node/costmap/dynamic_map"] as const;
const POSE_TOPICS = ["/amcl_pose", "/localizer_node/costmap/pose", "/odom", "/odometry/filtered"] as const;

function LidarPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [mapMsg, setMapMsg] = useState<any | null>(null);
  const [costmapMsg, setCostmapMsg] = useState<any | null>(null);
  const [lidarMsg, setLidarMsg] = useState<any | null>(null);
  const [robotPose, setRobotPose] = useState<RobotPose>({ x: 0, y: 0, yaw: 0 });
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  const [zoom, setZoom] = useState(1);
  const [offset, setOffset] = useState({ x: 0, y: 0 });
  const [rotation, setRotation] = useState(0);
  const [isDragging, setIsDragging] = useState(false);
  const [lastMousePos, setLastMousePos] = useState<{ x: number; y: number } | null>(null);
  const hasLocalizationPoseRef = useRef(false);

  // --- Suscripciones ---
  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const msgs = renderState.currentFrame ?? [];
      let bestPose: PoseCandidate | undefined;

      for (const msgEvent of msgs) {
        const isLocalizationTopic = msgEvent.topic === "/amcl_pose" || msgEvent.topic === "/localizer_node/costmap/pose";
        const isOdometryTopic = msgEvent.topic === "/odom" || msgEvent.topic === "/odometry/filtered";

        switch (msgEvent.topic) {
          case "/map":
          case "/maps_manager_node/costmap/map":
            setMapMsg(msgEvent.message);
            break;
          case "/global_costmap/costmap":
          case "/maps_manager_node/costmap/dynamic_map":
            setCostmapMsg(msgEvent.message);
            break;
          case "/rslidar_points":
            setLidarMsg(msgEvent.message);
            break;
          case "/amcl_pose":
          case "/localizer_node/costmap/pose":
          case "/odom":
          case "/odometry/filtered":
            const m = msgEvent.message as any;
            if (m?.pose?.pose?.position && m?.pose?.pose?.orientation) {
              if (isLocalizationTopic) {
                hasLocalizationPoseRef.current = true;
              }
              if (isOdometryTopic && hasLocalizationPoseRef.current) {
                break;
              }

              const p = m.pose.pose.position;
              const o = m.pose.pose.orientation;
              const yaw = Math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y*o.y + o.z*o.z));
              const priority = isLocalizationTopic ? 2 : 1;
              if (!bestPose || priority >= bestPose.priority) {
                bestPose = { pose: { x: p.x, y: p.y, yaw }, priority };
              }
            }
            break;
        }
      }

      if (bestPose) {
        setRobotPose(bestPose.pose);
      }
    };

    context.watch("currentFrame");
    context.subscribe([
      ...MAP_TOPICS.map((topic) => ({ topic })),
      ...COSTMAP_TOPICS.map((topic) => ({ topic })),
      { topic: "/rslidar_points" },
      ...POSE_TOPICS.map((topic) => ({ topic })),
    ]);
  }, [context]);

  useEffect(() => { renderDone?.(); }, [renderDone]);

  // --- Dibujar ---
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !mapMsg) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save();

    ctx.translate(canvas.width / 2 + offset.x, canvas.height / 2 + offset.y);
    ctx.scale(zoom, zoom);
    ctx.rotate(rotation);
    ctx.scale(1, -1);

    // Mapa y costmap
    drawOccupancyGrid(ctx, mapMsg, mapMsg.info.origin.position, mapMsg.info.resolution, "map");
    if (costmapMsg) drawOccupancyGrid(ctx, costmapMsg, mapMsg.info.origin.position, mapMsg.info.resolution, "costmap");

    // LiDAR relativo al robot
    if (lidarMsg && mapMsg) drawLidarPoints(ctx, lidarMsg, mapMsg.info.origin.position, mapMsg.info.resolution, robotPose);

    ctx.restore();
  }, [mapMsg, costmapMsg, lidarMsg, robotPose, zoom, offset, rotation]);

  // --- Zoom / Pan / Rotación (igual que antes) ---
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const handleWheel = (e: WheelEvent) => { e.preventDefault(); setZoom((z) => Math.min(Math.max(z * (e.deltaY<0?1.1:0.9), 0.2), 10)); };
    canvas.addEventListener("wheel", handleWheel); 
    return () => canvas.removeEventListener("wheel", handleWheel);
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const handleMouseDown = (e: MouseEvent) => { setIsDragging(true); setLastMousePos({ x: e.clientX, y: e.clientY }); };
    const handleMouseMove = (e: MouseEvent) => {
      if (!isDragging || !lastMousePos) return;
      setOffset((o) => ({ x: o.x + e.clientX - lastMousePos.x, y: o.y + e.clientY - lastMousePos.y }));
      setLastMousePos({ x: e.clientX, y: e.clientY });
    };
    const handleMouseUp = () => setIsDragging(false);

    canvas.addEventListener("mousedown", handleMouseDown);
    canvas.addEventListener("mousemove", handleMouseMove);
    window.addEventListener("mouseup", handleMouseUp);
    return () => { canvas.removeEventListener("mousedown", handleMouseDown); canvas.removeEventListener("mousemove", handleMouseMove); window.removeEventListener("mouseup", handleMouseUp); };
  }, [isDragging, lastMousePos]);

  useEffect(() => {
    const handleKey = (e: KeyboardEvent) => { if (e.key==="a") setRotation((r)=>r-0.1); if (e.key==="d") setRotation((r)=>r+0.1); };
    window.addEventListener("keydown", handleKey);
    return () => window.removeEventListener("keydown", handleKey);
  }, []);

  return (
    <div style={{padding:"0.5rem",background:"#1a1a1a",color:"#eee",height:"100%"}}>
      <canvas ref={canvasRef} style={{border:"1px solid #555",width:"100%",height:"90%",cursor:isDragging?"grabbing":"grab"}} />
    </div>
  );
}

// --- Dibujar LiDAR relativo a robot ---
function drawLidarPoints(ctx: CanvasRenderingContext2D, msg: any, mapOrigin: {x:number,y:number}, resolution: number, robotPose: RobotPose) {
  if (!msg?.data || !msg?.fields) return;
  const buf: Uint8Array = msg.data instanceof Uint8Array ? msg.data : new Uint8Array(msg.data);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  const fields: PointField[] = msg.fields;
  const xField = fields.find(f=>f.name==="x")?.offset??0;
  const yField = fields.find(f=>f.name==="y")?.offset??4;
  const step = msg.point_step ?? 16;
  const totalPoints = (msg.height??1)*(msg.width??1);

  const pointRadius = 0.3;
  const maxRange = 3;

  const cosYaw = Math.cos(robotPose.yaw);
  const sinYaw = Math.sin(robotPose.yaw);

  ctx.save();
  ctx.fillStyle="#fc0000ff";

  for(let i=0;i<totalPoints;i++){
    const off=i*step;



    
    if(off+Math.max(xField,yField)+4>buf.length) break;
    const x=dv.getFloat32(off+xField,true);
    const y=dv.getFloat32(off+yField,true);
    if(!isFinite(x)||!isFinite(y)||Math.sqrt(x*x+y*y)>maxRange) continue;
    const xRot = x*cosYaw - y*sinYaw;
    const yRot = x*sinYaw + y*cosYaw;
    const px=(xRot+robotPose.x-mapOrigin.x)/resolution;
    const py=(yRot+robotPose.y-mapOrigin.y)/resolution;
    ctx.beginPath();
    ctx.arc(px,py,pointRadius,0,Math.PI*2);
    ctx.fill();
  }
  ctx.restore();
}

// --- Dibujar Occupancy Grid ---
function drawOccupancyGrid(ctx:any,msg:any,mapOrigin:{x:number,y:number},mapResolution:number,type:"map"|"costmap"){
  const width=msg.info.width;
  const height=msg.info.height;
  const res=msg.info.resolution;
  const origin=msg.info.origin.position;
  const data=msg.data;
  const imageData=ctx.createImageData(width,height);

  for(let i=0;i<data.length;i++){
    const v=data[i];
    let r=0,g=0,b=0,a=255;
    if(type==="map"){ if(v<0) a=0; else if(v===0) r=g=b=255; else if(v===100) r=g=b=0; else r=g=b=200;}
    else{ if(v<0) a=0; else if(v===0){r=180;g=255;b=255;a=80;} else if(v===100){r=255;g=120;b=120;a=120;} else {r=120;g=150;b=255;a=100;}}
    const idx=i*4; imageData.data[idx]=r; imageData.data[idx+1]=g; imageData.data[idx+2]=b; imageData.data[idx+3]=a;
  }

  const tempCanvas=document.createElement("canvas");
  tempCanvas.width=width; tempCanvas.height=height;
  tempCanvas.getContext("2d")?.putImageData(imageData,0,0);
  const scale=res/mapResolution;
  const px=(origin.x-mapOrigin.x)/mapResolution;
  const py=(origin.y-mapOrigin.y)/mapResolution;
  ctx.drawImage(tempCanvas,px,py,width*scale,height*scale);
}

// --- Export ---
export function initLidarPanel(context: PanelExtensionContext):()=>void{
  const root=createRoot(context.panelElement);
  root.render(<LidarPanel context={context}/>);
  return ()=>root.unmount();
}
