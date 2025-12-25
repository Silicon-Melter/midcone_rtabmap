#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class GhostTF(Node):
    def __init__(self):
        super().__init__('ghost_tf_broadcaster')
        
        # Subscribe to the Visual Odometry message
        self.subscription = self.create_subscription(
            Odometry,
            '/vo_odom_topic',
            self.handle_odom,
            10)
            
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("Ghost TF Bridge Started: /vo_odom_topic -> base_link_ghost")

    def handle_odom(self, msg):
        t = TransformStamped()

        # We keep the same timestamp to ensure sync
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # 'vo_odom'
        
        # CRITICAL: We rename the child frame to avoid conflict!
        t.child_frame_id = 'base_link_ghost' 

        # Copy Position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Copy Rotation
        t.transform.rotation = msg.pose.pose.orientation

        # Publish the "Ghost" transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GhostTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()