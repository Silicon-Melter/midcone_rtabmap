import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class PX4OdomConverter(Node):
    def __init__(self):
        super().__init__('px4_odom_converter')

        # QoS for PX4 (Best Effort is required)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber (PX4 Data)
        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.callback,
            qos_profile
        )

        # Publisher (Standard ROS 2 Odom)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def callback(self, msg):
        # 1. Create Standard Odometry Message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # PX4 uses NED (North-East-Down), ROS uses ENU (East-North-Up)
        # We need to rotate the coordinates: x=y, y=x, z=-z
        # (Simplified conversion, assumes flat optical flow)
        
        # Position
        odom.pose.pose.position.x = msg.position[1]  # East
        odom.pose.pose.position.y = msg.position[0]  # North
        odom.pose.pose.position.z = -msg.position[2] # Up

        # Orientation (Quaternion)
        # PX4 (w, x, y, z) -> ROS (x, y, z, w) + NED/ENU rotation
        # Note: A proper quaternion rotation is complex. 
        # For simple flow, we pass it through but you might need a static transform 
        # for 'base_link_ned' depending on your URDF.
        odom.pose.pose.orientation.w = msg.q[0]
        odom.pose.pose.orientation.x = msg.q[1]
        odom.pose.pose.orientation.y = msg.q[2]
        odom.pose.pose.orientation.z = msg.q[3]

        # Publish Odom
        self.pub.publish(odom)

        # 2. Broadcast TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation = odom.pose.pose.position
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()