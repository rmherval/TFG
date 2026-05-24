import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL
from src.utils.gps_utils import latLonYaw2Geopose
from nav2_simple_commander.robot_navigator import BasicNavigator

import json
import paho.mqtt.client as mqtt
import queue
from threading import Thread
import time

class GpsWpCommander(Node):
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints received using MQTT
    """

    def __init__(self):
        super().__init__('mqtt_gps_wp_commander')
        self.navigator = BasicNavigator("basic_navigator")
        self.localizer = self.create_client(FromLL,  '/fromLL')

        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.mqtt_queue = queue.Queue()
        # self.create_timer(0.5, self.process_mqtt_queue)

        #MQTT with WebSocket 
        self.mqtt_client = mqtt.Client(transport="websockets")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        self.mqtt_client.connect("158.42.163.157", 9001, 60)
        self.mqtt_client.loop_start()

        self.process_mqtt_queue()

    def on_mqtt_connect (self, client, userdata, flags, rc):
        self.get_logger().info(f"Conectado a MQTT (WebSocket) con codigo {rc}")
        client.subscribe("location/coordinates")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            json_str = msg.payload.decode()
            waypoints = json.loads(json_str)
            self.get_logger().info(f"Recibidos {len(waypoints)} puntos por MQTT")
            self.mqtt_queue.put(waypoints)

        except Exception as e:
            self.get_logger().error(f"Error al procesar mensaje MQTT: {e}")

    def process_mqtt_queue(self):
        while True:
            try:
                msg = self.mqtt_queue.get(timeout=5)
            except queue.Empty:
                continue
            
            self.start_wpf(waypoints=msg)

            # Simulate slow processing
            time.sleep(0.5)  # Replace with actual processing logic
            self.mqtt_queue.task_done()
                    
    def start_wpf(self, waypoints):
        """
        Function to start the waypoint following
        """

        self.navigator.waitUntilNav2Active(localizer='controller_server')

        goal_poses = []

        for wp in waypoints:
            lat = wp["latitude"]
            lon = wp["longitude"]
            yaw = float(wp["yaw"])

            geo_pose = latLonYaw2Geopose(lat, lon, yaw)
            
            print("waypoints lat long " + str(geo_pose))

            req = FromLL.Request()
            req.ll_point.longitude = geo_pose.position.longitude
            req.ll_point.latitude = geo_pose.position.latitude
            req.ll_point.altitude = geo_pose.position.altitude

            log = 'long{:f}, lat={:f}, alt={:f}'.format(req.ll_point.longitude, req.ll_point.latitude, req.ll_point.altitude)
            self.get_logger().info(log)

            future = self.localizer.call_async(req)
            rclpy.spin_until_future_complete(self, future)


            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position = future.result().map_point
            pose_stamped.pose.orientation = geo_pose.orientation

            log = 'x={:f}, y={:f}, z={:f}'.format(future.result().map_point.x, future.result().map_point.y, future.result().map_point.z)
            self.get_logger().info(log)
            
            goal_poses.append(pose_stamped)

        self.get_logger().info("Enviando waypoints a Nav2")
        self.navigator.followWaypoints(goal_poses)
        print("wps completed successfully")

def main():
    rclpy.init()
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