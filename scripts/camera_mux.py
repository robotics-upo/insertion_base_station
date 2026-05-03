#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

class CameraMuxNode(Node):
    """
    Camera Multiplexer Node
    -----------------------
    Subscribes to multiple camera streams and forwards only one to the output topic
    based on a control signal from the GUI/Flight Panel.
    """
    def __init__(self):
        super().__init__('camera_mux_node')
        
        # --- PARAMETERS ---
        self.declare_parameter('default_focus', 0)
        self.declare_parameter('control_topic', '/gui/focus_drone')
        self.declare_parameter('output_topic', '/gui/focused_camera')
        self.declare_parameter('cam1_topic', '/drone1/camera/final_image')
        self.declare_parameter('cam2_topic', '/drone2/camera/final_image')
        
        self.current_focus = self.get_parameter('default_focus').value
        
        # QoS Profile optimized for high-throughput sensor data (Best Effort)
        qos_img = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # --- SUBSCRIBERS ---
        # Control topic to switch cameras (Default reliable QoS)
        self.focus_sub = self.create_subscription(
            Int32, 
            self.get_parameter('control_topic').value, 
            self.focus_callback, 
            10
        )
        
        # Camera inputs
        self.cam1_sub = self.create_subscription(
            Image, 
            self.get_parameter('cam1_topic').value, 
            self.cam1_callback, 
            qos_img
        )
        self.cam2_sub = self.create_subscription(
            Image, 
            self.get_parameter('cam2_topic').value, 
            self.cam2_callback, 
            qos_img
        )
        
        # --- PUBLISHERS ---
        self.focus_pub = self.create_publisher(
            Image, 
            self.get_parameter('output_topic').value, 
            qos_img
        )
        
        self.get_logger().info(f"Camera Mux Initialized. Defaulting to Drone {self.current_focus + 1}.")

    def focus_callback(self, msg):
        """Updates the active camera stream based on GUI input."""
        if self.current_focus != msg.data:
            self.current_focus = msg.data
            self.get_logger().info(f"Switched camera focus to Drone {self.current_focus + 1}")

    def cam1_callback(self, msg):
        """Forwards Drone 1 camera if it is currently selected."""
        if self.current_focus == 0:
            self.focus_pub.publish(msg)

    def cam2_callback(self, msg):
        """Forwards Drone 2 camera if it is currently selected."""
        if self.current_focus == 1:
            self.focus_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Camera Mux Node.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()