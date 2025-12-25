#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomFixer(Node):
    def __init__(self):
        super().__init__('odom_fixer_node')
        
        # Subscribe to the broken MAVROS odom
        self.sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.callback,
            10)
            
        # Publish the fixed odom
        self.pub = self.create_publisher(Odometry, '/mavros/odom_fixed', 10)
        self.get_logger().info("Odom Fixer Running: map->odom, cov=0.1")

    def callback(self, msg):
        # 1. FIX FRAME ID
        # We force the frame to be 'odom' so it fits the TF chain
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # 2. FIX COVARIANCE (The "Zero Trust" fix)
        # If covariance is 0, we give it a small value (0.1) so EKF uses it.
        # Pose Covariance (Position)
        if msg.pose.covariance[0] == 0.0:
            fixed_cov = [0.0] * 36
            fixed_cov[0] = 0.1  # X
            fixed_cov[7] = 0.1  # Y
            fixed_cov[14] = 0.1 # Z
            fixed_cov[21] = 0.05 # Roll
            fixed_cov[28] = 0.05 # Pitch
            fixed_cov[35] = 0.05 # Yaw
            msg.pose.covariance = fixed_cov

        # Twist Covariance (Velocity)
        if msg.twist.covariance[0] == 0.0:
            fixed_cov_twist = [0.0] * 36
            fixed_cov_twist[0] = 0.1
            fixed_cov_twist[7] = 0.1
            fixed_cov_twist[14] = 0.1
            fixed_cov_twist[21] = 0.05
            fixed_cov_twist[28] = 0.05
            fixed_cov_twist[35] = 0.05
            msg.twist.covariance = fixed_cov_twist

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()