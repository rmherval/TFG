import json
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from robot_localization.srv import FromLL
from nav2_simple_commander.robot_navigator import BasicNavigator

from src.utils.gps_utils import latLonYaw2Geopose


class GpsWpCommander(Node):

    def __init__(self):

        super().__init__('gps_wp_commander')

        self.get_logger().info("Iniciando GPS WP Commander...")

        # -------------------------------------------------
        # NAV2
        # -------------------------------------------------

        self.navigator = BasicNavigator("basic_navigator")

        # -------------------------------------------------
        # SERVICIO FROMLL
        # -------------------------------------------------

        self.localizer = self.create_client(
            FromLL,
            '/fromLL'
        )

        while not self.localizer.wait_for_service(timeout_sec=1.0):

            self.get_logger().info(
                'Esperando servicio /fromLL...'
            )

        self.get_logger().info(
            'Servicio /fromLL disponible'
        )

        # -------------------------------------------------
        # COLA DE WAYPOINTS
        # -------------------------------------------------

        self.goals_queue = deque()

        # -------------------------------------------------
        # SUBSCRIBER
        # -------------------------------------------------

        self.subscription = self.create_subscription(
            String,
            '/goals',
            self.goals_callback,
            10
        )

        # -------------------------------------------------
        # TIMER
        # -------------------------------------------------

        self.timer = self.create_timer(
            0.5,
            self.process_queue
        )

        self.processing = False

        self.get_logger().info(
            'Nodo listo. Escuchando /goals'
        )

    # =====================================================
    # CALLBACK TOPIC /goals
    # =====================================================

    def goals_callback(self, msg: String):

        try:

            waypoints = json.loads(msg.data)

            self.get_logger().info(
                f'Recibidos {len(waypoints)} waypoints'
            )

            self.goals_queue.append(waypoints)

        except Exception as e:

            self.get_logger().error(
                f'Error parseando JSON: {str(e)}'
            )

    # =====================================================
    # PROCESAR COLA
    # =====================================================

    def process_queue(self):

        if self.processing:
            return

        if len(self.goals_queue) == 0:
            return

        self.processing = True

        try:

            waypoints = self.goals_queue.popleft()

            self.start_wpf(waypoints)

        except Exception as e:

            self.get_logger().error(
                f'Error procesando waypoints: {str(e)}'
            )

        finally:

            self.processing = False

    # =====================================================
    # WAYPOINT FOLLOWER
    # =====================================================

    def start_wpf(self, waypoints):

        self.get_logger().info(
            'Esperando Nav2 activo...'
        )

        self.navigator.waitUntilNav2Active(localizer='controller_server')
        # self.navigator.lifecycleStartup()

        self.get_logger().info(
            'Nav2 activo'
        )

        goal_poses = []

        # -------------------------------------------------
        # CONVERTIR WAYPOINTS
        # -------------------------------------------------

        for i, wp in enumerate(waypoints):

            try:

                lat = float(wp["latitude"])
                lon = float(wp["longitude"])
                yaw = float(wp["yaw"])

                self.get_logger().info(
                    f'Waypoint {i}: '
                    f'lat={lat}, lon={lon}, yaw={yaw}'
                )

                # -----------------------------------------
                # ORIENTACION
                # -----------------------------------------

                geo_pose = latLonYaw2Geopose(
                    lat,
                    lon,
                    yaw
                )

                # -----------------------------------------
                # GPS -> MAP
                # -----------------------------------------

                req = FromLL.Request()

                req.ll_point.latitude = lat
                req.ll_point.longitude = lon
                req.ll_point.altitude = 0.0

                self.get_logger().info(
                    'Convirtiendo GPS -> MAP...'
                )

                self.pending_future = self.localizer.call_async(req)
                self.pending_wp = wp
                self.pending_future.add_done_callback(self.fromll_done)
                return

    def fromll_done(self, future):

        try:
            result = future.result()

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()

            pose_stamped.pose.position = result.map_point

            wp = self.pending_wp

            geo_pose = latLonYaw2Geopose(
                wp["latitude"],
                wp["longitude"],
                wp["yaw"]
            )

            pose_stamped.pose.orientation = geo_pose.orientation

            self.goal_poses.append(pose_stamped)

        except Exception as e:
            self.get_logger().error(f"fromLL error: {e}")
            # -------------------------------------------------
            # ENVIAR A NAV2
            # -------------------------------------------------

            if len(goal_poses) == 0:

                self.get_logger().warn(
                    'No hay waypoints válidos'
                )

                return

            self.get_logger().info(
                f'Enviando {len(goal_poses)} waypoints a Nav2'
            )

            # self.navigator.goThroughPoses(goal_poses)

            self.get_logger().info(
                'Waypoints enviados correctamente'
            )


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = GpsWpCommander()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        executor.shutdown()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()