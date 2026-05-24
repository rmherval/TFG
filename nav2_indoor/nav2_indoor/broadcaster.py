#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTfBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        # Crea el broadcaster de transformaciones
        self.tf_broadcaster = TransformBroadcaster(self)
        # Suscríbete al tópico /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info('Odom → TF broadcaster iniciado')

    def odom_callback(self, msg: Odometry):
        # Crea transformStamped
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        # El frame padre: normalmente "odom"
        t.header.frame_id = msg.header.frame_id        # p.ej. "odom"
        # El frame hijo: normalmente la base del robot, p.ej. "base_link"
        t.child_frame_id = msg.child_frame_id          # p.ej. "base_link"

        # Posición desde el mensaje odometry
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Orientación (cuaternion) desde el mensaje odometry
        t.transform.rotation = msg.pose.pose.orientation

        # Publica la transformación
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

