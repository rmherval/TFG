import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ScanBridge(Node):

    def __init__(self):
        super().__init__('scan_bridge')

        # QoS del subscriber (LiDAR normalmente BEST_EFFORT)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS del publisher (para EasyNav -> RELIABLE)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(
            LaserScan,
            '/scan_effort',
            pub_qos
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sub_qos
        )

        self.get_logger().info("Scan bridge running: /scan -> /scan_effort (RELIABLE)")

    def scan_callback(self, msg):
        # Re-publish exact same message
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()