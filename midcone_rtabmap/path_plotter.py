import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry  # Added Odometry import
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PathPlotter(Node):
    def __init__(self):
        super().__init__('path_plotter')
        
        # QoS Profile: MAVROS usually publishes Best Effort, so we match it
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to the Odometry topic instead of Pose
        # Common topics: '/mavros/local_position/odom' (Fused) or '/mavros/odometry/out' (Raw VIO)
        self.sub = self.create_subscription(
            Odometry, 
            '/mavros/local_position/odom', 
            self.listener_callback, 
            qos_best_effort
        )
        
        # Publish a Path topic for RViz
        self.pub = self.create_publisher(Path, '/visual_path', 10)
        
        # The history buffer
        self.path_msg = Path()
        
        self.get_logger().info("Path Plotter Started: Listening to /mavros/local_position/odom")

    def listener_callback(self, msg):
        # 'msg' is now type: nav_msgs/Odometry
        
        # We need to create a PoseStamped because Path only accepts PoseStamped
        current_pose = PoseStamped()
        
        # 1. Copy the Header (Timestamp and Frame ID are critical)
        current_pose.header = msg.header
        
        # 2. Extract the Pose
        # Odometry message structure is: msg.pose.pose (PoseWithCovariance -> Pose)
        current_pose.pose = msg.pose.pose
        
        # Update the main Path header to match the latest frame
        self.path_msg.header = msg.header
        
        # Append the new converted pose to the history
        self.path_msg.poses.append(current_pose)
        
        # Limit path length to prevent memory issues
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses.pop(0)

        self.pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()