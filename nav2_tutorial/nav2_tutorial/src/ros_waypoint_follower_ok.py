#!/usr/bin/env python3
import rclpy, math, json, argparse, paho.mqtt.client as mqtt
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Quaternion
from robot_localization.srv import FromLL
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import ComputePathToPose
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPose

config = {}

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose

class MqttPointCommander(Node):
    def __init__(self):
        super().__init__('ros_point_commander')
        self.navigator = BasicNavigator()
        self.localizer = self.create_client(FromLL, '/fromLL')
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        #Inicializar variables
        self.busy = self.reachable = self.arrived = False
        self.last_point = None
        self.queue = []

        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando /fromLL...')
        while not self.planner.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando planner_server...')

        #Conexion MQTT
        # self.mqtt = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        # self.mqtt.on_message = self.on_mqtt_message
        # self.mqtt.connect(f"{config['host']}", int(config['port']))
        # self.mqtt.subscribe(f"location/coordinates/{config['robot_id']}")
        # self.mqtt.loop_start()
        self.goal_sub = self.create_subscription(NavSatFix,'/goal_latlong',self.on_goal_msg,10)

    def on_goal_msg(self, msg: NavSatFix):
        try:
            x = msg.latitude
            y = msg.longitude

            self.get_logger().info(
                f"Punto ROS recibido: lat={x}, lon={y}"
            )

        except Exception as e:
            self.get_logger().error(f"Error leyendo NavSatFix: {e}")
            return
        if self.last_point is not None:
            last_x, last_y = self.last_point
            if math.isclose(x, last_x, abs_tol=1e-6) and math.isclose(y, last_y, abs_tol=1e-6):
                self.get_logger().warn("Punto recibido es igual al último, ignorando.")
                # if self.arrived:
                #     self.mqtt.publish(f"arrived/{config['robot_id']}", 'true')
                # if self.reachable:
                #     self.mqtt.publish(f"reachable/{config['robot_id']}", 'true')
                # return
        if self.busy:
            self.get_logger().warn("Robot ocupado, ignorando nuevo punto.")
            return

        self.queue.append((x, y))
        self.get_logger().info(f"Nuevo punto encolado: {x}, {y}")

        if not self.busy:
            self.process_next()

    def process_next(self):
        if not self.queue:
            return

        x, y = self.queue.pop(0)
        self.busy = True
        self.last_point = (x, y)
        self.send_to_robot(x, y)

    def send_to_robot(self, x: float, y: float):
        geo_pose = latLonYaw2Geopose(x, y, 0.0)

        req = FromLL.Request()
        req.ll_point.latitude = geo_pose.position.latitude
        req.ll_point.longitude = geo_pose.position.longitude
        req.ll_point.altitude = geo_pose.position.altitude

        self.get_logger().info(
            f"Llamando /fromLL: lat={req.ll_point.latitude}, lon={req.ll_point.longitude}"
        )

        future = self.localizer.call_async(req)
        future.add_done_callback(lambda f: self.on_fromll_response(f, geo_pose))

    def on_fromll_response(self, future, geo_pose):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Error al llamar /fromLL: {str(e)}")
            self.busy = False
            self.process_next()
            return

        map_pt = result.map_point

        self.get_logger().info(
            f"Enviando meta a Nav2: x={map_pt.x}, y={map_pt.y}"
        )

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = map_pt
        pose.pose.orientation = geo_pose.orientation

        self.check_goal_reachability(pose)

    def check_goal_reachability(self, goal_pose: PoseStamped):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_msg.use_start = False

        self.get_logger().info("Llamando acción ComputePathToPose para verificar si se puede llegar...")

        send_goal_future = self.planner.send_goal_async(goal_msg)

        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Goal rechazado por el planner.")
                # self.mqtt.publish(f"reachable/{config['robot_id']}", 'false')
                self.busy = False
                self.process_next()
                return

            self.get_logger().info("Goal aceptado. Esperando resultado del planificador...")

            get_result_future = goal_handle.get_result_async()

            def result_callback(result_fut):
                result = result_fut.result().result
                if len(result.path.poses) > 1:
                    self.get_logger().info("Meta alcanzable: path generado.")
                    # self.mqtt.publish(f"reachable/{config['robot_id']}", 'true')
                    self.reachable = True
                    self.send_goal_to_nav2(goal_pose)
                else:
                    self.get_logger().warn("No se pudo generar un path a la meta.")
                    # self.mqtt.publish(f"reachable/{config['robot_id']}", 'false')
                    self.reachable  = False
                    self.busy = False
                    self.process_next()

            get_result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

    def send_goal_to_nav2(self, pose: PoseStamped):
        self.navigator.waitUntilNav2Active(localizer='controller_server')
        self.navigator.goToPose(pose)
        self.get_logger().info("Navegando hacia la meta...")

        def check_complete():
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("El robot ha llegado al destino.")
                    # self.mqtt.publish(f"arrived/{config['robot_id']}", 'true')
                    self.arrived = True
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn("La navegación fue cancelada.")
                    # self.mqtt.publish(f"arrived/{config['robot_id']}", 'false')
                    self.arrived = False
                elif result == TaskResult.FAILED:
                    self.get_logger().error("La navegación falló.")
                    # self.mqtt.publish(f"arrived/{config['robot_id']}", 'false')
                    self.arrived = False
                self.busy = False
                self.process_next()
                self.nav_check_timer.cancel()
        self.nav_check_timer = self.create_timer(0.5, check_complete)
   
def main():
    parser = argparse.ArgumentParser(description="Ejemplo de argumentos en Python")
    parser.add_argument("--robot_id", type=str, default="001", help="ID del robot")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Host del broker MQTT")
    parser.add_argument("--port", type=int, default=1880, help="Port del broker MQTT")
    # args = parser.parse_args()
    args, _ = parser.parse_known_args()

    # Guardar en config global
    config["robot_id"] = args.robot_id
    config["host"] = args.host
    config["port"] = args.port
    
    # rclpy.init()
    rclpy.init(args=None)
    node = MqttPointCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
