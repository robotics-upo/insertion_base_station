#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraDecompressor(Node):
    """
    Node responsible for decompressing image streams from a ROS bag or live source.
    It subscribes to a CompressedImage topic, decodes it using OpenCV, and republishes
    it as a standard raw Image message with a corrected Frame ID.
    """

    def __init__(self):
        super().__init__('camera_decompressor')
        
        # Declare a parameter for the output frame ID
        self.declare_parameter('output_frame', 'camera_link')
        self.output_frame = self.get_parameter('output_frame').value

        # QoS configuration optimized for high-throughput sensor data (Best Effort)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # SUBSCRIBER: Listens to the compressed image stream
        # Note: Ensure this topic matches the one in your bag file or camera driver
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.listener_callback,
            qos_profile
        )
        
        # PUBLISHER: Outputs the raw, decompressed image
        self.pub = self.create_publisher(Image, '/camera/final_image', 10)

        # Bridge for converting between ROS messages and OpenCV images
        self.bridge = CvBridge()

        self.get_logger().info("Camera Decompressor Node initialized successfully.")

    def listener_callback(self, msg):
        """
        Callback function to process incoming compressed images.
        """

        try:
            # 1. Decompress the image data using OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # 2. Convert the OpenCV image to a standard ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            
            # 3. Header Management:
            # Retain the original timestamp for synchronization but enforce the Frame ID.
            # This ensures compatibility with the TF tree in RViz.
            img_msg.header = msg.header
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = self.output_frame 
            
            self.pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to decompress image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraDecompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()