#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sys

class RadarDataInspector(Node):
    """
    Diagnostic tool designed to inspect the schema of incoming PointCloud2 messages.
    
    Functionality:
    - Subscribes to the radar topic.
    - Captures the first received message.
    - Prints the field layout (offsets, names, datatypes) to the console.
    - Terminates immediately after inspection.
    
    Usage:
    Run this node to reverse-engineer the binary format of a new sensor driver.
    """

    def __init__(self):
        super().__init__('radar_data_inspector')
        
        # Subscribe to the target topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/radar/PointCloudDetection',
            self.listener_callback,
            10
        )
        
        # Mapping ROS 2 PointField datatypes to readable strings for easier debugging
        self.type_mappings = {
            PointField.INT8:    'INT8',
            PointField.UINT8:   'UINT8',
            PointField.INT16:   'INT16',
            PointField.UINT16:  'UINT16',
            PointField.INT32:   'INT32',
            PointField.UINT32:  'UINT32',
            PointField.FLOAT32: 'FLOAT32',
            PointField.FLOAT64: 'FLOAT64'
        }

        self.get_logger().info("Listening for incoming PointCloud2 messages...")
        self.get_logger().info("Waiting for data...")

    def listener_callback(self, msg):
        """
        Callback triggered upon receiving the first message.
        Analyzes structure and shuts down the node.
        """
        print("\n" + "="*60)
        print(f"   [ANALYSIS RESULT] POINTCLOUD2 STRUCTURE DETECTED   ")
        print("="*60)
        print(f" Width: {msg.width} | Height: {msg.height} | Is Dense: {msg.is_dense}")
        print(f" Point Step: {msg.point_step} bytes | Row Step: {msg.row_step} bytes")
        print("-" * 60)
        print(f" {'INDEX':<5} | {'FIELD NAME':<15} | {'OFFSET':<8} | {'DATATYPE':<10}")
        print("-" * 60)
        
        # Iterate through fields and print details
        for i, field in enumerate(msg.fields):
            dtype_str = self.type_mappings.get(field.datatype, 'UNKNOWN')
            print(f" {i:<5} | {field.name:<15} | {field.offset:<8} | {dtype_str} ({field.datatype})")
            
        print("="*60 + "\n")
        
        # Graceful shutdown after single capture
        self.get_logger().info("Inspection complete. Shutting down.")
        raise SystemExit # Forces the loop in main() to break

def main(args=None):
    rclpy.init(args=args)
    inspector = RadarDataInspector()
    
    try:
        rclpy.spin(inspector)
    except SystemExit:
        # Expected behavior when the node shuts itself down
        pass
    except KeyboardInterrupt:
        pass
    finally:
        inspector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()