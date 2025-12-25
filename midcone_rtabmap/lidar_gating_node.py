import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class LidarGatingNode(Node):
    def __init__(self):
        super().__init__('lidar_gating_node')

        # --- PARAMETERS ---
        # Max vertical speed (m/s) allowed before we cut off LiDAR
        self.declare_parameter('max_vz', 0.2)
        # Max statistical distance (sigma) allowed for a jump
        self.declare_parameter('mahalanobis_thresh', 4.0)
        
        self.max_vz = self.get_parameter('max_vz').value
        self.mahalanobis_thresh = self.get_parameter('mahalanobis_thresh').value

        # --- SUBSCRIBERS ---
        # 1. The Trusted Source (Your EKF output: fused VO + IMU)
        self.ekf_sub = self.create_subscription(
            Odometry, '/odom', self.ekf_callback, 10)

        # 2. The "Untrusted" Candidate (Your LiDAR ICP)
        self.lidar_sub = self.create_subscription(
            Odometry, '/odom_lidar_raw', self.lidar_callback, 10)

        # --- PUBLISHER ---
        # We only publish here if the data is good
        self.fused_pub = self.create_publisher(
            Odometry, '/odom_lidar_filtered', 10)

        # State Variables
        self.ekf_vz = 0.0
        self.ekf_pos = np.zeros(2) # x, y
        self.ekf_cov = np.eye(2)   # 2x2 covariance
        self.ekf_initialized = False

    def ekf_callback(self, msg):
        """Update our 'Ground Truth' belief from the main EKF"""
        self.ekf_vz = msg.twist.twist.linear.z
        self.ekf_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        
        # Extract 2x2 Covariance (X, Y) from the 6x6 matrix (index 0 and 7 of flattened array?)
        # ROS 2 covariance is row-major 6x6 (36 floats)
        # Indices: 0=XX, 1=XY, 6=YX, 7=YY
        cov_full = np.array(msg.pose.covariance).reshape(6,6)
        self.ekf_cov = cov_full[0:2, 0:2]
        self.ekf_initialized = True

    def lidar_callback(self, msg):
        if not self.ekf_initialized:
            return # Wait for EKF to boot

        # --- CHECK 1: ALTITUDE STABILITY ---
        # If drone is moving up/down, LiDAR geometry is changing (degeneracy risk)
        if abs(self.ekf_vz) > self.max_vz:
            # excessive z-velocity, ignore lidar
            return 

        # --- CHECK 2: MAHALANOBIS GATING ---
        # Does the LiDAR say we are somewhere completely different from the VO?
        lidar_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        innovation = lidar_pos - self.ekf_pos
        
        # Combined uncertainty: S = P_ekf + R_lidar
        lidar_cov_full = np.array(msg.pose.covariance).reshape(6,6)
        lidar_cov_2d = lidar_cov_full[0:2, 0:2]
        
        S = self.ekf_cov + lidar_cov_2d
        
        try:
            S_inv = np.linalg.inv(S)
            # Distance squared = d^T * S^-1 * d
            mahalanobis_sq = innovation.T @ S_inv @ innovation
            distance = np.sqrt(mahalanobis_sq)

            if distance < self.mahalanobis_thresh:
                # PASSED: Safe to fuse
                self.fused_pub.publish(msg)
            else:
                self.get_logger().warn(f"LiDAR Rejected: Jump detected (Dist={distance:.2f})")
                
        except np.linalg.LinAlgError:
            pass # Singular matrix, ignore

def main(args=None):
    rclpy.init(args=args)
    node = LidarGatingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()