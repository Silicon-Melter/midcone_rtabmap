#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class QoSBridge(Node):
    def __init__(self):
        super().__init__('qos_bridge_node')

        # 1. LISTEN to the "Dirty" Volatile topic (From the Bag)
        # We will remap the bag to publish here.
        self.sub = self.create_subscription(
            CameraInfo,
            '/camera_info_bag_source',  # <--- NEW TEMP NAME
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        )

        # 2. PUBLISH the "Clean" Latched topic (To the Node)
        # This is the original name the node expects.
        self.pub = self.create_publisher(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info', # <--- TARGET NAME
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.get_logger().info('QoS Bridge Started: /camera_info_bag_source -> /camera/camera/aligned_depth_to_color/camera_info')

    def listener_callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()