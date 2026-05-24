#!/usr/bin/env python3
"""
Analizador simple de navegación para TFG.
Lee automáticamente goals de /goal_pose y calcula métricas para cada uno.

Métricas calculadas:
- Tiempo de navegación
- Distancia recorrida
- Velocidad media y máxima
- Eficiencia de trayecto
- Si llegó al destino

Uso:
    python3 simple_analyzer.py <bagfile>

Ejemplo:
    python3 simple_analyzer.py ./bags/recorrido_corto
"""

import json
import math
import sqlite3
import sys
import os
from bisect import bisect_right
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    rosbag2_py = None
    deserialize_message = None
    get_message = None

try:
    from rosbags.rosbag2 import Reader as RosbagsReader
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    RosbagsReader = None
    Stores = None
    get_typestore = None


class SimpleNavigationAnalyzer:
    """Analizador simple: lee goals de /goal_pose automáticamente."""

    # Punto inicial conocido en frame map para este recorrido
    INITIAL_MAP_POSE = (1.7056117057800293, -2.324835777282715, 0.0)

    # Tolerancia para considerar dos goals iguales (metros)
    GOAL_DEDUP_TOLERANCE = 0.2

    # Tolerancia para detectar que el robot ha llegado (metros)
    ARRIVAL_TOLERANCE = 0.3

    # Velocidad mínima para considerar que se está moviendo
    MIN_VELOCITY = 0.01

    def __init__(self, bag_path: str):
        self.bag_path = Path(bag_path)
        if not self.bag_path.exists():
            raise FileNotFoundError(f"No se encontró el bag: {self.bag_path}")

        self.messages_by_topic: Dict[str, List[Tuple[int, Any]]] = {}
        self._tf_static_cache: Dict[Tuple[str, str], Optional[Tuple[float, float, float]]] = {}
        self._tf_time_index: Dict[Tuple[str, str], Tuple[List[int], List[Tuple[float, float, float]]]] = {}
        self._load_messages()
        self._build_tf_time_index()

    def _detect_storage_id(self) -> str:
        """Detecta si es MCAP o SQLite3."""
        bag_files = list(self.bag_path.iterdir())
        if any(f.suffix.lower() == ".mcap" for f in bag_files if f.is_file()):
            return "mcap"
        return "sqlite3"

    def _load_messages(self) -> None:
        """Carga mensajes del rosbag con el mejor método disponible."""
        if rosbag2_py is not None:
            self._load_with_rosbag2()
            return

        if RosbagsReader is not None:
            self._load_with_rosbags()
            return

        if self._detect_storage_id() != "sqlite3":
            raise RuntimeError("Necesitas rosbag2_py para leer MCAP")

        self._load_from_sqlite()

    def _load_with_rosbag2(self) -> None:
        """Lee con rosbag2_py."""
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=str(self.bag_path))
        converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")
        reader.open(storage_options, converter_options)

        topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()

            if topic_name in ["/odom", "/goal_pose", "/tf", "/tf_static"]:
                if topic_name not in self.messages_by_topic:
                    self.messages_by_topic[topic_name] = []

                topic_type = topic_types.get(topic_name)
                try:
                    msg = self._deserialize(data, topic_type)
                    self.messages_by_topic[topic_name].append((timestamp, msg))
                except Exception:
                    pass

    def _load_with_rosbags(self) -> None:
        """Lee con rosbags (alternativa)."""
        typestore = get_typestore(Stores.LATEST)

        with RosbagsReader(self.bag_path) as reader:
            for conn, timestamp, rawdata in reader.messages():
                if conn.topic in ["/odom", "/goal_pose", "/tf", "/tf_static"]:
                    if conn.topic not in self.messages_by_topic:
                        self.messages_by_topic[conn.topic] = []
                    try:
                        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                        self.messages_by_topic[conn.topic].append((timestamp, msg))
                    except Exception:
                        pass

    def _load_from_sqlite(self) -> None:
        """Lee directamente de SQLite3 (último recurso)."""
        db_files = list(self.bag_path.glob("*.db3"))
        if not db_files:
            raise FileNotFoundError("No hay archivo .db3 en el bag")

        conn = sqlite3.connect(str(db_files[0]))
        cursor = conn.cursor()

        for topic_name in ["/odom", "/goal_pose", "/tf", "/tf_static"]:
            cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
            result = cursor.fetchone()
            if not result:
                continue

            topic_id = result[0]
            cursor.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
                (topic_id,)
            )

            self.messages_by_topic[topic_name] = []
            for timestamp, data in cursor.fetchall():
                try:
                    if topic_name == "/odom":
                        msg = self._deserialize(data, "nav_msgs/Odometry")
                    elif topic_name == "/goal_pose":
                        msg = self._deserialize(data, "geometry_msgs/PoseStamped")
                    else:
                        msg = self._deserialize(data, "tf2_msgs/TFMessage")
                    self.messages_by_topic[topic_name].append((timestamp, msg))
                except Exception:
                    pass

        conn.close()

    def _build_tf_time_index(self) -> None:
        """Construye un índice temporal por par de frames para /tf."""
        tf_msgs = self.messages_by_topic.get("/tf", [])
        if not tf_msgs:
            return

        indexed: Dict[Tuple[str, str], List[Tuple[int, Tuple[float, float, float]]]] = {}

        for timestamp, msg in tf_msgs:
            transforms = getattr(msg, "transforms", [])
            for tf in transforms:
                child = (getattr(tf, "child_frame_id", "") or "").lstrip("/")
                parent = (getattr(tf.header, "frame_id", "") if hasattr(tf, "header") else "").lstrip("/")
                tf_data = self._extract_tf_data(tf)
                if tf_data is None:
                    continue
                indexed.setdefault((child, parent), []).append((timestamp, tf_data))

        for key, values in indexed.items():
            values.sort(key=lambda item: item[0])
            self._tf_time_index[key] = (
                [ts for ts, _ in values],
                [tf_data for _, tf_data in values],
            )

    def _deserialize(self, data: bytes, msg_type: str) -> Any:
        """Deserializa un mensaje ROS 2."""
        if deserialize_message is None or get_message is None:
            raise RuntimeError("Necesitas rosbag2_py/rclpy instalado")

        msg_class = get_message(msg_type)
        return deserialize_message(data, msg_class)

    @staticmethod
    def _get_pose(message: Any) -> Optional[Tuple[float, float, float]]:
        """Extrae (x, y, z) de un mensaje Odometry o PoseStamped."""
        try:
            pose = message.pose.pose
            x = float(pose.position.x)
            y = float(pose.position.y)
            z = float(pose.position.z)
            return (x, y, z)
        except Exception:
            try:
                pose = message.pose
                x = float(pose.position.x)
                y = float(pose.position.y)
                z = float(pose.position.z)
                return (x, y, z)
            except Exception:
                return None

    @staticmethod
    def _get_velocity(message: Any) -> Optional[Tuple[float, float, float]]:
        """Extrae velocidades (vx, vy, vz) de un mensaje Odometry."""
        try:
            twist = message.twist.twist
            vx = float(twist.linear.x)
            vy = float(twist.linear.y)
            vz = float(twist.linear.z)
            return (vx, vy, vz)
        except Exception:
            return None

    def _extract_unique_goals(self) -> List[Tuple[int, float, float, float]]:
        """
        Extrae goals únicos de /goal_pose, eliminando repeticiones.
        Como ros2 topic pub publica el mismo goal repetidamente,
        agrupa por distancia y devuelve solo goals distintos.
        """
        goal_messages = self.messages_by_topic.get("/goal_pose", [])
        unique_goals: List[Tuple[int, float, float, float]] = []

        for timestamp, msg in goal_messages:
            pose = self._get_pose(msg)
            if pose is None:
                continue

            x, y, z = pose

            if not unique_goals:
                unique_goals.append((timestamp, x, y, z))
                continue

            last = unique_goals[-1]
            dist = math.sqrt((x - last[1])**2 + (y - last[2])**2)

            if dist > self.GOAL_DEDUP_TOLERANCE:
                unique_goals.append((timestamp, x, y, z))

        return unique_goals

    @staticmethod
    def _distance_2d(p1: Tuple[float, float, float], p2: Tuple[float, float, float]) -> float:
        """Distancia euclidiana en 2D."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx*dx + dy*dy)

    @staticmethod
    def _linear_speed(vx: float, vy: float, vz: float) -> float:
        """Magnitud de velocidad lineal en 2D."""
        return math.sqrt(vx*vx + vy*vy)

    def _extract_tf_transform(self, child_frame: str, parent_frame: str) -> Optional[Tuple[float, float, float]]:
        """Extrae la transformación (tx, ty, yaw) entre dos frames."""
        result = self._find_tf_direct(child_frame, parent_frame)
        if result:
            return result

        result_inv = self._find_tf_direct(parent_frame, child_frame)
        if result_inv:
            tx, ty, yaw = result_inv
            return self._invert_transform(tx, ty, yaw)

        print(f"  ❌ No encontrada transformación {parent_frame}->{child_frame}")
        return None

    def _extract_tf_transform_at_time(
        self,
        child_frame: str,
        parent_frame: str,
        timestamp: int,
    ) -> Optional[Tuple[float, float, float]]:
        """Extrae la transformación más reciente válida para un instante dado."""
        result = self._find_tf_direct_at_time(child_frame, parent_frame, timestamp)
        if result:
            return result

        result_inv = self._find_tf_direct_at_time(parent_frame, child_frame, timestamp)
        if result_inv:
            tx, ty, yaw = result_inv
            return self._invert_transform(tx, ty, yaw)

        return None

    def _find_tf_direct_at_time(
        self,
        child_frame: str,
        parent_frame: str,
        timestamp: int,
    ) -> Optional[Tuple[float, float, float]]:
        """Busca la TF directa más reciente en o antes de timestamp."""
        child_key = child_frame.lstrip("/")
        parent_key = parent_frame.lstrip("/")

        cache_key = (child_key, parent_key)
        static_cached = self._tf_static_cache.get(cache_key)
        if cache_key in self._tf_static_cache:
            if static_cached is not None:
                return static_cached

        tf_static_msgs = self.messages_by_topic.get("/tf_static", [])
        if tf_static_msgs:
            for _, msg in tf_static_msgs:
                transforms = getattr(msg, "transforms", [])
                for tf in transforms:
                    child = (getattr(tf, "child_frame_id", "") or "").lstrip("/")
                    parent = (getattr(tf.header, "frame_id", "") if hasattr(tf, "header") else "").lstrip("/")
                    if child == child_key and parent == parent_key:
                        tf_data = self._extract_tf_data(tf)
                        self._tf_static_cache[cache_key] = tf_data
                        return tf_data

        self._tf_static_cache[cache_key] = None

        indexed = self._tf_time_index.get(cache_key)
        if not indexed:
            return None

        timestamps, transforms = indexed
        pos = bisect_right(timestamps, timestamp) - 1
        if pos < 0:
            return None

        return transforms[pos]

    def _find_tf_direct(self, child_frame: str, parent_frame: str) -> Optional[Tuple[float, float, float]]:
        """Busca una transformación específica parent->child."""
        tf_static_msgs = self.messages_by_topic.get("/tf_static", [])

        if tf_static_msgs:
            for _, msg in tf_static_msgs:
                transforms = getattr(msg, "transforms", [])
                for tf in transforms:
                    child = getattr(tf, "child_frame_id", "")
                    parent = getattr(tf.header, "frame_id", "") if hasattr(tf, "header") else ""

                    if child == child_frame and parent == parent_frame:
                        return self._extract_tf_data(tf)

        tf_msgs = self.messages_by_topic.get("/tf", [])

        if tf_msgs:
            for _, msg in tf_msgs[-50:]:
                transforms = getattr(msg, "transforms", [])
                for tf in transforms:
                    child = getattr(tf, "child_frame_id", "")
                    parent = getattr(tf.header, "frame_id", "") if hasattr(tf, "header") else ""

                    if child == child_frame and parent == parent_frame:
                        return self._extract_tf_data(tf)

        return None

    @staticmethod
    def _extract_tf_data(tf_msg: Any) -> Optional[Tuple[float, float, float]]:
        """Extrae (tx, ty, yaw) de un mensaje TF."""
        try:
            trans = getattr(tf_msg, "transform", None)
            transl = getattr(trans, "translation", None)
            rot = getattr(trans, "rotation", None)

            tx = float(getattr(transl, "x", 0))
            ty = float(getattr(transl, "y", 0))

            qx = float(getattr(rot, "x", 0))
            qy = float(getattr(rot, "y", 0))
            qz = float(getattr(rot, "z", 0))
            qw = float(getattr(rot, "w", 1))
            yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

            return (tx, ty, yaw)
        except Exception:
            return None

    @staticmethod
    def _invert_transform(tx: float, ty: float, yaw: float) -> Tuple[float, float, float]:
        """Invierte una transformación 2D (tx, ty, yaw)."""
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        tx_inv = -(tx * cos_y + ty * sin_y)
        ty_inv = -(-tx * sin_y + ty * cos_y)
        yaw_inv = -yaw

        return (tx_inv, ty_inv, yaw_inv)

    def _transform_point(self, point: Tuple[float, float, float],
                         tf_transform: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Transforma un punto usando una transformación TF."""
        if tf_transform is None:
            return point

        tx, ty, yaw = tf_transform
        x, y, z = point

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        x_rot = x * cos_y - y * sin_y
        y_rot = x * sin_y + y * cos_y

        x_final = x_rot + tx
        y_final = y_rot + ty

        return (x_final, y_final, z)

    def _map_to_odom(self, point: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Convierte un punto en frame map a frame odom."""
        tf_map_to_odom = self._extract_tf_transform("map", "odom")
        if tf_map_to_odom:
            return self._transform_point(point, tf_map_to_odom)
        return point

    def _odom_to_map_at(self, timestamp: int, point: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Convierte un punto en odom a map usando la TF válida en ese instante."""
        tf_odom_to_map = self._extract_tf_transform_at_time("odom", "map", timestamp)
        if tf_odom_to_map:
            return self._transform_point(point, tf_odom_to_map)
        return point

    # ------------------------------------------------------------------
    # MÉTODO PRINCIPAL: analyze()
    # Lógica corregida para N goals secuenciales desde INITIAL_MAP_POSE
    # ------------------------------------------------------------------
    def analyze(self) -> Dict[str, Any]:
        """
        Analiza la navegación secuencial entre goals.

        Esquema de segmentos:
          Segmento 1: INITIAL_MAP_POSE  → Goal 1
          Segmento 2: Goal 1            → Goal 2
          Segmento N: Goal N-1          → Goal N

        Los goals se leen directamente de /goal_pose, eliminando
        duplicados producidos por 'ros2 topic pub'.
        """
        odom_messages = self.messages_by_topic.get("/odom", [])
        if not odom_messages:
            raise RuntimeError("No se encontraron mensajes de /odom")

        unique_goals = self._extract_unique_goals()
        if not unique_goals:
            raise RuntimeError("No se encontraron goals en /goal_pose")

        # Extraer todas las posiciones de odometría
        poses: List[Tuple[int, float, float, float]] = []
        for timestamp, msg in odom_messages:
            pose = self._get_pose(msg)
            if pose:
                poses.append((timestamp, *pose))

        if not poses:
            raise RuntimeError("No se pudieron extraer posiciones de /odom")

        print(f"\n📊 Analizando navegación secuencial")
        print("=" * 70)
        print(f"  Total mensajes /odom: {len(odom_messages)}")
        print(f"  Posiciones únicas:    {len(poses)}")
        print(f"  Goals detectados:     {len(unique_goals)}")
        print(f"\n  Punto de inicio (map): x={self.INITIAL_MAP_POSE[0]:.3f}, "
              f"y={self.INITIAL_MAP_POSE[1]:.3f}")
        print(f"\n  Goals encontrados (frame map):")
        for i, (ts, x, y, z) in enumerate(unique_goals):
            print(f"    Goal {i+1}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # Transformar la odometría a frame map usando la TF válida en cada instante.
        # Esto evita comparar un trayecto largo en odom contra goals definidos en map.
        poses_map: List[Tuple[int, float, float, float]] = []
        for timestamp, x, y, z in poses:
            poses_map.append((timestamp, *self._odom_to_map_at(timestamp, (x, y, z))))

        # Waypoints en frame map: el inicio conocido y los goals tal cual se publicaron.
        waypoints_map: List[Tuple[float, float, float]] = [self.INITIAL_MAP_POSE]
        for _, x, y, z in unique_goals:
            waypoints_map.append((x, y, z))

        print(f"\n  Waypoints en frame map:")
        labels = ["Inicio"] + [f"Goal {i+1}" for i in range(len(unique_goals))]
        for label, wp in zip(labels, waypoints_map):
            print(f"    {label}: x={wp[0]:.3f}, y={wp[1]:.3f}")

        # Analizar cada segmento consecutivo en frame map
        journey_results = []
        for i in range(len(unique_goals)):
            start_wp = waypoints_map[i]       # origen del segmento
            end_wp   = waypoints_map[i + 1]   # destino del segmento

            result = self._analyze_segment(
                poses=poses_map,
                start_wp=start_wp,
                end_wp=end_wp,
                segment_number=i + 1,
            )
            journey_results.append(result)

        print("\n" + "=" * 70)
        print(f"\n📋 Resumen global:")
        print(f"   Segmentos analizados: {len(journey_results)}")
        print(f"   Distancia total:      {sum(r['distance_traveled_meters'] for r in journey_results):.3f} m")
        print(f"   Tiempo total:         {sum(r['total_time_seconds'] for r in journey_results):.2f} s")
        arrived_count = sum(1 for r in journey_results if r["arrived"])
        print(f"   Goals alcanzados:     {arrived_count}/{len(journey_results)}")

        return {
            "goals_detected": len(unique_goals),
            "total_segments": len(journey_results),
            "segments": journey_results,
            "total_distance_all_segments": sum(r["distance_traveled_meters"] for r in journey_results),
            "total_time_all_segments": sum(r["total_time_seconds"] for r in journey_results),
            "goals_reached": arrived_count,
        }

    # ------------------------------------------------------------------
    # Análisis de un segmento entre dos waypoints (ambos en frame map)
    # ------------------------------------------------------------------
    def _analyze_segment(
        self,
        poses: List[Tuple[int, float, float, float]],
        start_wp: Tuple[float, float, float],
        end_wp: Tuple[float, float, float],
        segment_number: int,
    ) -> Dict[str, Any]:
        """
        Analiza el segmento número 'segment_number'.

        Busca el tramo de poses que va desde cerca de start_wp hasta
        cerca de end_wp, y calcula las métricas sobre ese tramo.
        """
        print(f"\n  ▶️  Segmento {segment_number}:")
        print(f"      Desde (map): x={start_wp[0]:.3f}, y={start_wp[1]:.3f}")
        print(f"      Hasta (map): x={end_wp[0]:.3f},   y={end_wp[1]:.3f}")

        # --- Encontrar índice de inicio del segmento ---
        # Usar la pose más cercana al waypoint de inicio (mejor robustez frente a offsets)
        min_dist_start = None
        min_idx_start = 0
        for i, (_, x, y, z) in enumerate(poses):
            d = self._distance_2d((x, y, z), start_wp)
            if min_dist_start is None or d < min_dist_start:
                min_dist_start = d
                min_idx_start = i

        # Si la pose más cercana está muy lejos, avisar y usarla de todas formas
        if min_dist_start is not None and min_dist_start > max(self.ARRIVAL_TOLERANCE, 1.0):
            print(f"      ⚠️  Inicio lejos de odom: dist_min={min_dist_start:.3f} m (pose idx {min_idx_start}). Usando la pose más cercana como inicio.")

        start_idx = min_idx_start

        # --- Encontrar índice de fin del segmento ---
        # Primero intentar detectar llegada tras start_idx con la tolerancia definida
        end_idx = None
        for i in range(start_idx, len(poses)):
            _, x, y, z = poses[i]
            if self._distance_2d((x, y, z), end_wp) < self.ARRIVAL_TOLERANCE:
                end_idx = i
                break

        # Si no encontramos una llegada explícita, tomar la pose más cercana al goal después de start_idx
        if end_idx is None:
            min_dist_end = None
            min_idx_end = len(poses) - 1
            for i in range(start_idx, len(poses)):
                _, x, y, z = poses[i]
                d = self._distance_2d((x, y, z), end_wp)
                if min_dist_end is None or d < min_dist_end:
                    min_dist_end = d
                    min_idx_end = i

            if min_dist_end is not None:
                print(f"      ⚠️  No se detectó llegada exacta; usando pose más cercana al goal: dist_min={min_dist_end:.3f} m (pose idx {min_idx_end})")
                end_idx = min_idx_end
                arrived = (min_dist_end <= self.ARRIVAL_TOLERANCE * 1.5)
            else:
                end_idx = len(poses) - 1
                arrived = False
        else:
            arrived = True

        segment_poses = poses[start_idx: end_idx + 1]

        if len(segment_poses) < 2:
            print(f"      ⚠️  Pocas poses en este segmento ({len(segment_poses)}). Usando todas las poses disponibles.")
            segment_poses = poses

        return self._calculate_metrics(segment_poses, end_wp, arrived, segment_number)

    # ------------------------------------------------------------------
    # Cálculo de métricas sobre un conjunto de poses
    # ------------------------------------------------------------------
    def _calculate_metrics(
        self,
        journey_poses: List[Tuple[int, float, float, float]],
        target_pose: Tuple[float, float, float],
        arrived: bool,
        segment_number: int,
    ) -> Dict[str, Any]:
        """Calcula todas las métricas para un conjunto de poses."""

        # Tiempo
        start_time = journey_poses[0][0]
        end_time   = journey_poses[-1][0]
        total_time_s = (end_time - start_time) / 1e9

        # Distancia recorrida
        distance_traveled = 0.0
        for i in range(1, len(journey_poses)):
            p1 = journey_poses[i-1][1:]
            p2 = journey_poses[i][1:]
            distance_traveled += self._distance_2d(p1, p2)

        # Distancia en línea recta inicio→fin del segmento
        straight_distance = self._distance_2d(
            journey_poses[0][1:], journey_poses[-1][1:]
        )

        # Velocidades desde /odom
        velocities = []
        odom_messages = self.messages_by_topic.get("/odom", [])
        for _, msg in odom_messages:
            vel = self._get_velocity(msg)
            if vel:
                velocities.append(self._linear_speed(*vel))

        mean_velocity     = distance_traveled / total_time_s if total_time_s > 0 else 0.0
        max_velocity      = max(velocities) if velocities else 0.0
        avg_cmd_velocity  = sum(velocities) / len(velocities) if velocities else 0.0

        # Eficiencia y sobrerrecorrido
        efficiency = straight_distance / distance_traveled if distance_traveled > 0 else 0.0
        overshoot  = distance_traveled - straight_distance

        # Precisión: distancia del último punto al objetivo
        real_precision = self._distance_2d(journey_poses[-1][1:], target_pose)

        # Confirmación adaptativa de llegada
        actually_arrived = arrived or (real_precision <= self.ARRIVAL_TOLERANCE * 1.5)

        print(f"      ⏱️  Tiempo:           {total_time_s:.2f} s")
        print(f"      📍 Distancia real:   {distance_traveled:.3f} m")
        print(f"      📏 Distancia recta:  {straight_distance:.3f} m")
        print(f"      ↗️  Sobrerrecorrido:  {overshoot:.3f} m")
        print(f"      🚀 Vel. media:       {mean_velocity:.3f} m/s")
        print(f"      🚀 Vel. máxima:      {max_velocity:.3f} m/s")
        print(f"      ✅ Eficiencia:       {efficiency:.1%}")
        print(f"      🎯 Precisión:        {real_precision:.3f} m")
        print(f"      🎯 Llegó:            {'Sí ✓' if actually_arrived else 'No ✗'}")

        return {
            "segment": segment_number,
            "total_time_seconds": total_time_s,
            "distance_traveled_meters": distance_traveled,
            "straight_distance_meters": straight_distance,
            "overshoot_meters": overshoot,
            "mean_velocity_ms": mean_velocity,
            "mean_cmd_velocity_ms": avg_cmd_velocity,
            "max_velocity_ms": max_velocity,
            "efficiency": efficiency,
            "arrived": actually_arrived,
            "real_precision_meters": real_precision,
            "poses_count": len(journey_poses),
            "start_pose": {
                "x": journey_poses[0][1],
                "y": journey_poses[0][2],
                "z": journey_poses[0][3],
            },
            "end_pose": {
                "x": journey_poses[-1][1],
                "y": journey_poses[-1][2],
                "z": journey_poses[-1][3],
            },
            "target_pose": {
                "x": target_pose[0],
                "y": target_pose[1],
                "z": target_pose[2],
            },
        }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    bag_path = sys.argv[1]

    try:
        analyzer = SimpleNavigationAnalyzer(bag_path)
        report = analyzer.analyze()

        report_dir = Path(bag_path)
        if not os.access(report_dir, os.W_OK):
            report_dir = Path.cwd()

        report_file = report_dir / "analysis_report.json"
        with open(report_file, "w") as f:
            json.dump(report, f, indent=2)

        print(f"\n💾 Reporte guardado en: {report_file}")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()