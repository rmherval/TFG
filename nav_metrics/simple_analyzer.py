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
    # Nota: si el robot no llega exactamente, se aumentará automáticamente
    ARRIVAL_TOLERANCE = 0.5  # Aumentado de 0.5 a 1.0m
    
    # Velocidad mínima para considerar que se está moviendo
    MIN_VELOCITY = 0.01

    def __init__(self, bag_path: str):
        self.bag_path = Path(bag_path)
        if not self.bag_path.exists():
            raise FileNotFoundError(f"No se encontró el bag: {self.bag_path}")

        self.messages_by_topic: Dict[str, List[Tuple[int, Any]]] = {}
        self._load_messages()

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
                    else:  # /tf o /tf_static
                        msg = self._deserialize(data, "tf2_msgs/TFMessage")
                    self.messages_by_topic[topic_name].append((timestamp, msg))
                except Exception:
                    pass

        conn.close()

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
            # Intenta con Odometry
            pose = message.pose.pose
            x = float(pose.position.x)
            y = float(pose.position.y)
            z = float(pose.position.z)
            return (x, y, z)
        except Exception:
            try:
                # Intenta con PoseStamped
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

            # Si es el primer goal
            if not unique_goals:
                unique_goals.append((timestamp, x, y, z))
                continue

            # Comparar con el último goal registrado
            last = unique_goals[-1]
            dist = math.sqrt((x - last[1])**2 + (y - last[2])**2)

            # Si es lo suficientemente distinto, es un nuevo goal
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
        """
        Extrae la transformación (tx, ty) entre dos frames.
        Busca en /tf_static primero, luego en /tf.
        Retorna (tx, ty, yaw) o None si no encuentra.
        """
        # Primero intentar buscar la transformación directa
        result = self._find_tf_direct(child_frame, parent_frame)
        if result:
            return result
        
        # Si no existe, buscar la inversa
        result_inv = self._find_tf_direct(parent_frame, child_frame)
        if result_inv:
            # Invertir la transformación
            tx, ty, yaw = result_inv
            return self._invert_transform(tx, ty, yaw)
        
        print(f"  ❌ No encontrada transformación {parent_frame}->{child_frame}")
        return None

    def _find_tf_direct(self, child_frame: str, parent_frame: str) -> Optional[Tuple[float, float, float]]:
        """Busca una transformación específica parent->child."""
        # Buscar en /tf_static primero
        tf_static_msgs = self.messages_by_topic.get("/tf_static", [])
        
        if tf_static_msgs:
            for _, msg in tf_static_msgs:
                transforms = getattr(msg, "transforms", [])
                for tf in transforms:
                    child = getattr(tf, "child_frame_id", "")
                    parent = getattr(tf.header, "frame_id", "") if hasattr(tf, "header") else ""
                    
                    if child == child_frame and parent == parent_frame:
                        return self._extract_tf_data(tf)
        
        # Buscar en /tf (últimos mensajes)
        tf_msgs = self.messages_by_topic.get("/tf", [])
        
        if tf_msgs:
            for _, msg in tf_msgs[-50:]:  # Últimos 50 mensajes
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
            
            # Extraer yaw del quaternión
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
        
        # Invertir: rotar el vector traslación con rotación inversa
        tx_inv = -(tx * cos_y + ty * sin_y)
        ty_inv = -(-tx * sin_y + ty * cos_y)
        yaw_inv = -yaw
        
        return (tx_inv, ty_inv, yaw_inv)

    def _transform_point(self, point: Tuple[float, float, float], 
                        tf_transform: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Transforma un punto usando una transformación TF.
        point: (x, y, z)
        tf_transform: (tx, ty, yaw)
        """
        if tf_transform is None:
            return point
        
        tx, ty, yaw = tf_transform
        x, y, z = point
        
        # Rotar el punto según yaw
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        
        x_rot = x * cos_y - y * sin_y
        y_rot = x * sin_y + y * cos_y
        
        # Trasladar
        x_final = x_rot + tx
        y_final = y_rot + ty
        z_final = z
        
        return (x_final, y_final, z_final)

    def _map_to_odom(self, point: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Convierte un punto en map a odom cuando existe la TF correspondiente."""
        tf_map_to_odom = self._extract_tf_transform("map", "odom")
        if tf_map_to_odom:
            return self._transform_point(point, tf_map_to_odom)
        return point

    def analyze(self) -> Dict[str, Any]:
        """
        Analiza los viajes entre goals.
        Lee automáticamente goals de /goal_pose y calcula métricas para cada segmento.
        """
        
        odom_messages = self.messages_by_topic.get("/odom", [])
        if not odom_messages:
            raise RuntimeError("No se encontraron mensajes de /odom")

        unique_goals = self._extract_unique_goals()
        if not unique_goals:
            raise RuntimeError("No se encontraron goals en /goal_pose")

        # Extraer todas las posiciones
        poses: List[Tuple[int, float, float, float]] = []
        for timestamp, msg in odom_messages:
            pose = self._get_pose(msg)
            if pose:
                poses.append((timestamp, *pose))

        if not poses:
            raise RuntimeError("No se pudieron extraer posiciones de /odom")

        print(f"\n📊 Analizando navegación")
        print("=" * 70)
        print(f"  Total mensajes /odom: {len(odom_messages)}")
        print(f"  Posiciones únicas: {len(poses)}")
        print(f"  Goals detectados: {len(unique_goals)}")
        print(f"\n  Goals encontrados:")
        for i, (ts, x, y, z) in enumerate(unique_goals):
            print(f"    Goal {i+1}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # Analizar cada segmento de navegación
        journey_results = []

        # El primer goal es el punto de partida
        initial_goal = unique_goals[0]
        
        # Si hay múltiples goals, analizar cada segmento
        if len(unique_goals) > 1:
            for i in range(1, len(unique_goals)):
                prev_goal = unique_goals[i-1]
                curr_goal = unique_goals[i]
                
                result = self._analyze_segment(poses, prev_goal, curr_goal, i)
                journey_results.append(result)
        else:
            # Si hay solo un goal, el viaje es desde la posición inicial hasta ese goal
            # Convertir el inicio conocido en map al mismo frame que las poses
            start_pose = self._map_to_odom(self.INITIAL_MAP_POSE)
            
            result = self._analyze_segment_simple(poses, start_pose, unique_goals[0])
            journey_results.append(result)

        print("\n" + "=" * 70)
        
        return {
            "goals_detected": len(unique_goals),
            "total_segments": len(journey_results),
            "segments": journey_results,
            "total_distance_all_segments": sum(r["distance_traveled_meters"] for r in journey_results),
            "total_time_all_segments": sum(r["total_time_seconds"] for r in journey_results),
        }

    def _analyze_segment(self, poses: List[Tuple[int, float, float, float]], 
                        start_goal: Tuple[int, float, float, float],
                        end_goal: Tuple[int, float, float, float],
                        segment_number: int) -> Dict[str, Any]:
        """Analiza un segmento de navegación entre dos goals."""
        
        print(f"\n  ▶️  Analizando segmento {segment_number}:")
        start_goal_odom = self._map_to_odom(start_goal[1:])
        end_goal_odom = self._map_to_odom(end_goal[1:])
        print(f"      Desde (map): x={start_goal[1]:.3f}, y={start_goal[2]:.3f}")
        print(f"      Desde (odom): x={start_goal_odom[0]:.3f}, y={start_goal_odom[1]:.3f}")
        print(f"      Hasta (map): x={end_goal[1]:.3f}, y={end_goal[2]:.3f}")
        print(f"      Hasta (odom): x={end_goal_odom[0]:.3f}, y={end_goal_odom[1]:.3f}")

        # Encontrar rango de posiciones para este segmento
        # Desde cuando estaba cerca del goal inicial
        start_idx = None
        for i, (_, x, y, z) in enumerate(poses):
            dist = self._distance_2d((x, y, z), start_goal_odom)
            if dist < self.ARRIVAL_TOLERANCE:
                start_idx = i
                break

        if start_idx is None:
            start_idx = 0

        # Hasta cuando llegó cerca del goal final
        end_idx = None
        for i in range(start_idx, len(poses)):
            _, x, y, z = poses[i]
            dist = self._distance_2d((x, y, z), end_goal_odom)
            if dist < self.ARRIVAL_TOLERANCE:
                end_idx = i
                break

        if end_idx is None:
            end_idx = len(poses) - 1
            arrived = False
        else:
            arrived = True

        journey_poses = poses[start_idx:end_idx+1]

        # Calcular métricas
        return self._calculate_metrics(journey_poses, end_goal_odom, arrived, segment_number)

    def _analyze_segment_simple(self, poses: List[Tuple[int, float, float, float]],
                               start_pose: Tuple[float, float, float],
                               end_goal: Tuple[int, float, float, float]) -> Dict[str, Any]:
        """Analiza un segmento simple (single goal)."""
        
        print(f"\n  ▶️  Analizando navegación hacia goal único:")
        print(f"      Inicio (map): x={self.INITIAL_MAP_POSE[0]:.3f}, y={self.INITIAL_MAP_POSE[1]:.3f}, z={self.INITIAL_MAP_POSE[2]:.3f}")
        start_pose_odom = self._map_to_odom(self.INITIAL_MAP_POSE)
        print(f"      Inicio (odom): x={start_pose_odom[0]:.3f}, y={start_pose_odom[1]:.3f}, z={start_pose_odom[2]:.3f}")
        print(f"      Goal (map):  x={end_goal[1]:.3f}, y={end_goal[2]:.3f}, z={end_goal[3]:.3f}")
        
        # Intentar transformar el goal del frame map al frame odom
        tf_map_to_odom = self._extract_tf_transform("map", "odom")
        
        goal_for_comparison = end_goal[1:]
        if tf_map_to_odom:
            print(f"  ✓ Transformación map->odom encontrada: tx={tf_map_to_odom[0]:.3f}, ty={tf_map_to_odom[1]:.3f}, yaw={tf_map_to_odom[2]:.3f}")
            goal_transformed = self._transform_point(end_goal[1:], tf_map_to_odom)
            print(f"      Goal (odom): x={goal_transformed[0]:.3f}, y={goal_transformed[1]:.3f}, z={goal_transformed[2]:.3f}")
            goal_for_comparison = goal_transformed
        else:
            print(f"  ⚠️  No se encontró transformación map->odom. Usando goal en frame original.")
        
        # Debugger: mostrar distancia a cada pose
        print(f"\n  📊 DEBUG: Analizando {len(poses)} poses...")
        for i, (_, x, y, z) in enumerate(poses):
            dist = self._distance_2d((x, y, z), goal_for_comparison)

            # Mostrar cada 50 poses para no saturar output
            if i % max(1, len(poses)//5) == 0:
                print(f"      Pose {i}: x={x:.3f}, y={y:.3f} -> dist a goal: {dist:.3f}m")

        print(f"  ✓ Tolerancia de llegada: {self.ARRIVAL_TOLERANCE}m")
        print(f"  ✓ Threshold adaptativo: {self.ARRIVAL_TOLERANCE * 1.5}m")

        # Encontrar donde llega al goal
        end_idx = None
        for i, (_, x, y, z) in enumerate(poses):
            dist = self._distance_2d((x, y, z), goal_for_comparison)
            if dist < self.ARRIVAL_TOLERANCE:
                end_idx = i
                break

        if end_idx is None:
            end_idx = len(poses) - 1
            arrived = False
        else:
            arrived = True

        journey_poses = poses[:end_idx+1]

        return self._calculate_metrics(journey_poses, goal_for_comparison, arrived, 1, poses[-1][1:])

    def _calculate_metrics(self, journey_poses: List[Tuple[int, float, float, float]],
                          target_pose: Tuple[float, float, float],
                          arrived: bool,
                          segment_number: int,
                          precision_pose: Optional[Tuple[float, float, float]] = None) -> Dict[str, Any]:
        """Calcula todas las métricas para un segmento."""
        
        # Tiempo
        start_time = journey_poses[0][0]
        end_time = journey_poses[-1][0]
        total_time_s = (end_time - start_time) / 1e9

        # Distancia recorrida
        distance_traveled = 0.0
        for i in range(1, len(journey_poses)):
            p1 = journey_poses[i-1][1:]
            p2 = journey_poses[i][1:]
            distance_traveled += self._distance_2d(p1, p2)

        # Distancia en línea recta
        straight_distance = self._distance_2d(journey_poses[0][1:], journey_poses[-1][1:])

        # Velocidades
        velocities = []
        odom_messages = self.messages_by_topic.get("/odom", [])
        for _, msg in odom_messages:
            vel = self._get_velocity(msg)
            if vel:
                speed = self._linear_speed(*vel)
                velocities.append(speed)

        mean_velocity = distance_traveled / total_time_s if total_time_s > 0 else 0.0
        max_velocity = max(velocities) if velocities else 0.0
        avg_cmd_velocity = sum(velocities) / len(velocities) if velocities else 0.0

        # Eficiencia
        efficiency = straight_distance / distance_traveled if distance_traveled > 0 else 0.0
        overshoot = distance_traveled - straight_distance

        # Precisión real: error del último punto del bag respecto al objetivo
        if precision_pose is None:
            precision_pose = journey_poses[-1][1:]
        real_precision = self._distance_2d(precision_pose, target_pose)

        # Redeterminar si llegó basándose en la distancia mínima
        # Considerar que llegó si se acercó a menos de 1.5x la tolerancia
        adaptive_threshold = self.ARRIVAL_TOLERANCE * 1.5
        actually_arrived = real_precision <= adaptive_threshold

        print(f"      ⏱️  Tiempo:              {total_time_s:.2f} s")
        print(f"      📍 Distancia real:      {distance_traveled:.3f} m")
        print(f"      📏 Distancia recta:     {straight_distance:.3f} m")
        print(f"      ↗️  Sobrerrecorrido:     {overshoot:.3f} m")
        print(f"      🚀 Velocidad media:     {mean_velocity:.3f} m/s")
        print(f"      🚀 Velocidad máxima:    {max_velocity:.3f} m/s")
        print(f"      ✅ Eficiencia:          {efficiency:.1%}")
        print(f"      🎯 Precisión:           {real_precision:.3f} m")
        print(f"      🎯 Llegó:               {'Sí ✓' if actually_arrived else 'No ✗'}")

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
            }
        }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    bag_path = sys.argv[1]

    try:
        analyzer = SimpleNavigationAnalyzer(bag_path)
        report = analyzer.analyze()

        # Guardar reporte JSON
        report_file = Path(bag_path) / "analysis_report.json"
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
