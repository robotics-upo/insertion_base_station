#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFTimeRewriterNode(Node):
    """
    TF Time Rewriter Node
    ---------------------
    Subscribes to an old TF topic from a rosbag, updates the timestamp to the 
    current system time (Wall Time), and injects a specific namespace prefix.
    Crucially, it prevents prefixing global frames ('map', 'gps') to keep a 
    unified TF tree for multiple vehicles.
    """
    def __init__(self):
        super().__init__('tf_time_rewriter_node')
        
        # --- PARAMETERS ---
        self.declare_parameter('input_topic', '/tf_old')
        self.declare_parameter('output_topic', '/tf')
        self.declare_parameter('prefix', '') 
        
        self.in_topic = self.get_parameter('input_topic').value
        self.out_topic = self.get_parameter('output_topic').value
        
        # --- SUBSCRIBERS & PUBLISHERS ---
        self.subscription = self.create_subscription(
            TFMessage, self.in_topic, self.rewrite_callback, 10
        )
        self.publisher = self.create_publisher(TFMessage, self.out_topic, 10)
        
        # Logging
        prefix_log = self.get_parameter('prefix').value or "None"
        self.get_logger().info(
            f"TF Time Rewriter Started: {self.in_topic} -> {self.out_topic} (Prefix: '{prefix_log}')"
        )

    def rewrite_callback(self, msg):
        """Updates timestamps and applies namespace prefixes safely."""
        current_time = self.get_clock().now().to_msg()
        prefix = self.get_parameter('prefix').value
        
        # Global frames that should NEVER be prefixed
        global_frames = ['map', 'gps']
        
        for transform in msg.transforms:
            # 1. Update timestamp to current Wall Time (Fixes bag latency in RViz)
            transform.header.stamp = current_time
            
            # 2. Inject namespace prefix to separate TF trees
            if prefix:
                # Clean up leading slashes and existing prefixes just in case
                f_id = transform.header.frame_id.lstrip('/').replace('drone1/', '').replace('drone2/', '')
                c_id = transform.child_frame_id.lstrip('/').replace('drone1/', '').replace('drone2/', '')
                
                # Apply prefix ONLY if it's not a global frame
                if f_id not in global_frames:
                    transform.header.frame_id = prefix + f_id
                else:
                    transform.header.frame_id = f_id 
                    
                if c_id not in global_frames:
                    transform.child_frame_id = prefix + c_id
                else:
                    transform.child_frame_id = c_id 
                
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFTimeRewriterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down TF Time Rewriter Node.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()