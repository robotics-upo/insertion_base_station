#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFTimeRewriter(Node):
    """
    Node that intercepts Transform (TF) messages from a recorded bag file (on a separate topic),
    overwrites their timestamps with the current system time, and republishes them to the main /tf tree.
    
    Use Case:
    When playing back old ROS bags, RViz often discards TFs because they are "too old" relative
     to the current wall time. This node synchronizes the bag data to 'now'.
    
    Usage:
    Play your bag remaping /tf to /tf_old:
        ros2 bag play my_bag.mcap --remap /tf:=/tf_old
    """

    def __init__(self):
        super().__init__('tf_time_rewriter')
        
        # --- CONFIGURATION ---
        # Allow topic names to be configured via launch parameters
        self.declare_parameter('input_topic', '/tf_old')
        self.declare_parameter('output_topic', '/tf')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # --- SUBSCRIBER ---
        # Listens to the "old" data from the bag file
        self.subscription = self.create_subscription(
            TFMessage,
            input_topic,
            self.rewrite_callback,
            10
        )
            
        # --- PUBLISHER ---
        # Broadcasts the "fresh" data to the visualization tools (RViz)
        self.publisher = self.create_publisher(TFMessage, output_topic, 10)
        
        self.get_logger().info(f"TF Rewriter Active: Forwarding '{input_topic}' -> '{output_topic}' with current timestamps.")

    def rewrite_callback(self, msg):
        """
        Callback to process incoming TF messages.
        Updates the timestamp of every transform in the list to the current clock time.
        """
        # Capture current system time
        current_time = self.get_clock().now().to_msg()
        
        # Iterate through all transforms in the message (parent -> child links)
        for transform in msg.transforms:
            # Overwrite the 'stamp' field in the header
            transform.header.stamp = current_time
            
        # Republish the modified message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFTimeRewriter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down TF Rewriter.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()