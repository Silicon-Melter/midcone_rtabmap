import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2

class CameraRestamper(Node):
    def __init__(self):
        super().__init__('camera_restamper')
        
        self.bridge = CvBridge()
        
        # Best Effort is often needed for high-bandwidth sensors (cameras) over Wi-Fi
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # --- RGB ---
        self.pub_rgb = self.create_publisher(Image, '/camera/fixed', 10)
        self.sub_rgb = self.create_subscription(Image, '/camera', self.rgb_cb, qos)

        # --- DEPTH ---
        self.pub_depth = self.create_publisher(Image, '/depth_camera/fixed', 10)
        self.sub_depth = self.create_subscription(Image, '/depth_camera', self.depth_cb, qos)
        
        # --- INFO ---
        self.pub_info = self.create_publisher(CameraInfo, '/camera_info/fixed', 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera_info', self.info_cb, qos)

    def rgb_cb(self, msg):
        # 1. Fix Timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 2. Resize Image to match Depth (640x480)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize to match Depth Resolution exactly (640x480)
            resized_image = cv2.resize(cv_image, (640, 480))
            
            out_msg = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
            out_msg.header = msg.header # Copy new timestamp/frame_id
            self.pub_rgb.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def depth_cb(self, msg):
        # We assume Depth is already 640x480 if you are recording at that resolution.
        # Resizing depth maps with standard interpolation introduces errors (flying pixels), 
        # so we pass it through with just a timestamp fix.
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_depth.publish(msg)

    def info_cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # CRITICAL: Update Camera Info to match new resolution (640x480)
        # We calculate the scale factor based on the incoming resolution vs target (640x480)
        scale_x = 640.0 / msg.width
        scale_y = 480.0 / msg.height
        
        msg.width = 640
        msg.height = 480
        
        # Scale the Intrinsic Matrix (K)
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        msg.k[0] *= scale_x # fx
        msg.k[2] *= scale_x # cx
        msg.k[4] *= scale_y # fy
        msg.k[5] *= scale_y # cy
        
        # Scale Projection Matrix (P)
        msg.p[0] *= scale_x # fx
        msg.p[2] *= scale_x # cx
        msg.p[5] *= scale_y # fy
        msg.p[6] *= scale_y # cy
        
        self.pub_info.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraRestamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()