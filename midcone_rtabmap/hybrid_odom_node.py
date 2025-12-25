import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class HybridOdomNode(Node):
    def __init__(self):
        super().__init__('hybrid_odom_node')

        self.initial_rotation_set = False
        self.yaw_offset = 0.0
        self.start_x_diff = 0.0
        self.start_y_diff = 0.0

        # QoS Settings
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_mavros = self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.mavros_callback, qos_best_effort
        )
        self.sub_rgbd = self.create_subscription(
            Odometry, '/odom_rgbd', self.rgbd_callback, qos_best_effort
        )

        self.pub_hybrid = self.create_publisher(Odometry, '/odom_hybrid', 10)
        self.latest_rgbd = None
        
        self.get_logger().info("Hybrid Node: Frame ID forced to 'odom' to fix TF Tree.")

    def rgbd_callback(self, msg):
        self.latest_rgbd = msg

    def mavros_callback(self, mavros_msg):
        if self.latest_rgbd is None:
            return

        # 1. Get current Yaws
        m_yaw = self.get_yaw(mavros_msg.pose.pose.orientation)
        r_yaw = self.get_yaw(self.latest_rgbd.pose.pose.orientation)

        # 2. CALIBRATION (Align Rotation)
        if not self.initial_rotation_set:
            self.yaw_offset = m_yaw - r_yaw
            
            # Align positions
            r_x_rot, r_y_rot = self.rotate_vector(
                self.latest_rgbd.pose.pose.position.x, 
                self.latest_rgbd.pose.pose.position.y, 
                self.yaw_offset
            )
            self.start_x_diff = mavros_msg.pose.pose.position.x - r_x_rot
            self.start_y_diff = mavros_msg.pose.pose.position.y - r_y_rot
            
            self.initial_rotation_set = True
            return

        # 3. ROTATE RGBD VECTOR
        r_x_raw = self.latest_rgbd.pose.pose.position.x
        r_y_raw = self.latest_rgbd.pose.pose.position.y
        r_x_rot, r_y_rot = self.rotate_vector(r_x_raw, r_y_raw, self.yaw_offset)
        
        v_x_raw = self.latest_rgbd.twist.twist.linear.x
        v_y_raw = self.latest_rgbd.twist.twist.linear.y
        v_x_rot, v_y_rot = self.rotate_vector(v_x_raw, v_y_raw, self.yaw_offset)

        # 4. CONSTRUCT MESSAGE
        hybrid_msg = mavros_msg
        
        # --- CRITICAL FIX: FORCE FRAME IDS ---
        # Mavros sends 'map' or 'local_origin'. EKF wants 'odom'.
        # We MUST overwrite this, or EKF rejects the data -> No TF published.
        hybrid_msg.header.frame_id = 'odom'
        hybrid_msg.child_frame_id = 'base_link'
        # -------------------------------------

        # Overwrite Position
        hybrid_msg.pose.pose.position.x = r_x_rot + self.start_x_diff
        hybrid_msg.pose.pose.position.y = r_y_rot + self.start_y_diff
        
        # Overwrite Velocity
        hybrid_msg.twist.twist.linear.x = v_x_rot
        hybrid_msg.twist.twist.linear.y = v_y_rot

        # Mix Covariance (RGBD for X/Y, Mavros for rest)
        pose_cov = list(hybrid_msg.pose.covariance)
        repl_cov = list(self.latest_rgbd.pose.covariance)
        pose_cov[0] = repl_cov[0] # X
        pose_cov[7] = repl_cov[7] # Y
        hybrid_msg.pose.covariance = tuple(pose_cov)
        
        self.pub_hybrid.publish(hybrid_msg)

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def rotate_vector(self, x, y, theta):
        x_new = x * math.cos(theta) - y * math.sin(theta)
        y_new = x * math.sin(theta) + y * math.cos(theta)
        return x_new, y_new

def main(args=None):
    rclpy.init(args=args)
    node = HybridOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()