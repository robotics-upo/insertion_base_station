#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

class AssetLabelsNode(Node):
    """
    Asset Labels Node
    -----------------
    Publishes 3D floating text markers for RViz to label vehicles 
    (Drone 1, Drone 2, Boat) in the operational environment.
    """
    def __init__(self):
        super().__init__('asset_labels_node')
        
        # --- PARAMETERS ---
        self.declare_parameter('topic_name', '/gui/asset_labels')
        self.declare_parameter('publish_rate', 2.0) # Hz
        
        topic_name = self.get_parameter('topic_name').value
        rate_hz = self.get_parameter('publish_rate').value
        
        # --- ASSET CONFIGURATION ---
        # Format: 'frame_id': ('Label Text', Marker ID, Z-Offset)
        self.assets = {
            'drone1/base_link': ('DRONE 1', 0, 1.0),
            'drone2/base_link': ('DRONE 2', 1, 1.0),
            'boat/base_link':   ('BOAT',    2, 2.5)
        }

        # --- PUBLISHERS & TIMERS ---
        self.publisher = self.create_publisher(MarkerArray, topic_name, 10)
        
        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self.publish_labels)
        
        self.get_logger().info(f"Asset Labels Node Initialized. Publishing at {rate_hz} Hz.")

    def publish_labels(self):
        """Iterates through defined assets and publishes their markers."""
        msg = MarkerArray()
        
        for frame_id, config in self.assets.items():
            text, marker_id, z_offset = config
            msg.markers.append(self.create_marker(frame_id, text, marker_id, z_offset))
        
        self.publisher.publish(msg)

    def create_marker(self, frame_id, text, marker_id, z_offset):
        """Generates a TEXT_VIEW_FACING marker tied to a specific TF frame."""
        marker = Marker()
        
        # Header (Time=0 ensures RViz uses the latest available transform)
        marker.header.frame_id = frame_id
        marker.header.stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg()
        
        # Identity and Action
        marker.ns = "asset_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position (anchored directly above the vehicle frame)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = float(z_offset)
        marker.pose.orientation.w = 1.0
        
        # Appearance (Scale & Color)
        marker.scale.z = 0.6 
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Lifetime (Self-destructs if not updated recently)
        marker.lifetime.sec = 1 
        marker.text = text
        
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = AssetLabelsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Asset Labels Node.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()