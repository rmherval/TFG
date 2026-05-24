import json
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from robot_localization.srv import FromLL
from nav2_simple_commander.robot_navigator import BasicNavigator

from src.utils.gps_utils import latLonYaw2Geopose


class GpsWpCommander(Node):

    def __init__(self):

        super().__init__('gps_wp_commander')

        self.get_logger().info("GPS Waypoint Commander iniciado")

        # -----------------------------
        # NAV2
        # -----------------------------
        self.navigator = BasicNavigator("basic_navigator")
        # self.navigator.lifecycleStartup()

        # -----------------------------
        # SERVICIO GPS -> MAP
        # -----------------------------
        self.localizer = self.create_client(FromLL, '/fromLL')

        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Esperando /fromLL...")

        # -----------------------------
        # SUBSCRIPTION
        # -----------------------------
        self.subscription = self.create_subscription(
            String,
            '/goals',
            self.goals_callback,
            10
        )

        # -----------------------------
        # COLAS
        # -----------------------------
        self.queue = deque()

        self.current_goal_poses = []
        self.current_wp_index = 0
        self.waiting_for_fromll = False
        self.pending_wp = None

        # timer de control
        self.timer = self.create_timer(0.1, self.process)

        self.get_logger().info("Nodo listo")

    # =====================================================
    # CALLBACK /goals
    # =====================================================
    def goals_callback(self, msg: String):

        try:
            waypoints = json.loads(msg.data)
            self.queue.append(waypoints)

            self.get_logger().info(
                f"Recibidos {len(waypoints)} waypoints"
            )

        except Exception as e:
            self.get_logger().error(f"JSON error: {e}")

    # =====================================================
    # LOOP PRINCIPAL
    # =====================================================
    def process(self):

        # 1. Si no hay navegación activa, coger nuevo set
        if not self.current_goal_poses and self.queue:

            waypoints = self.queue.popleft()

            self.prepare_goals(waypoints)
            return

        # 2. Esperar conversiones async
        if self.waiting_for_fromll:
            return

        # 3. Si no hay goals activos
        if not self.current_goal_poses:
            return

        # 4. Ejecutar siguiente waypoint
        if self.current_wp_index >= len(self.current_goal_poses):

            self.get_logger().info("Trayectoria completada")
            self.current_goal_poses = []
            self.current_wp_index = 0
            return

        # enviar siguiente goal a Nav2 (una sola vez)
        if self.current_wp_index == 0:

            self.get_logger().info(f"Enviando waypoints a Nav2 current_goal_pose={self.current_goal_poses}")

            self.navigator.goThroughPoses(
                self.current_goal_poses
            )

        self.current_wp_index += 1

    # =====================================================
    # PREPARAR WAYPOINTS
    # =====================================================
    def prepare_goals(self, waypoints):

        self.current_goal_poses = []
        self.current_wp_index = 0

        self.convert_index = 0
        self.raw_waypoints = waypoints

        self.convert_next()

    # =====================================================
    # CONVERSIÓN ASYNC (NO BLOQUEANTE)
    # =====================================================
    def convert_next(self):

        if self.convert_index >= len(self.raw_waypoints):
            self.waiting_for_fromll = False
            return

        wp = self.raw_waypoints[self.convert_index]

        lat = float(wp["latitude"])
        lon = float(wp["longitude"])
        yaw = float(0.1)

        self.pending_wp = wp
        self.waiting_for_fromll = True

        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = 0.0

        future = self.localizer.call_async(req)
        future.add_done_callback(self.fromll_done)

    # =====================================================
    # CALLBACK FROMLL
    # =====================================================
    def fromll_done(self, future):

        try:
            result = future.result()

            wp = self.pending_wp

            geo_pose = latLonYaw2Geopose(
                float(wp["latitude"]),
                float(wp["longitude"]),
                float(wp["yaw"])
            )

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position = result.map_point
            pose.pose.orientation = geo_pose.orientation

            self.current_goal_poses.append(pose)

            self.get_logger().info(
                f"Waypoint convertido {self.convert_index}"
            )

        except Exception as e:
            self.get_logger().error(f"fromLL error: {e}")

        self.convert_index += 1
        self.waiting_for_fromll = False

        # siguiente waypoint
        self.convert_next()


def main(args=None):

    rclpy.init(args=args)

    node = GpsWpCommander()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()