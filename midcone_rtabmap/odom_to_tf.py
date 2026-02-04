import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# --- 1. ADD THESE IMPORTS ---
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # --- 2. DEFINE THE 'BEST EFFORT' QoS PROFILE ---
        # This allows the node to listen to sensors that might drop packets (like your bag)
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- 3. USE THE QoS PROFILE IN SUBSCRIPTION ---
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.handle_odom,
            qos_policy) # <--- Replaces the number '10'
            
        self.get_logger().info('OdomToTF Bridge Started (Best Effort Mode)')

    def handle_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()