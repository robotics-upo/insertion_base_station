#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import struct
import math
import sys

class RadarVisualizer(Node):
    """
    Node that processes raw Radar PointClouds to visualize RCS (Radar Cross Section) intensity.
    
    Features:
    1. Color Gradient: Maps RCS intensity to a color scale (Blue=Water, Green=Land).
    2. Near-field Filtering: Removes specular reflections (ghosts) caused by the water 
       surface directly beneath the sensor.
    3. Timestamp Synchronization: Overrides message timestamps to fix visualization lag 
       in RViz during bag playback.
    """

    def __init__(self):
        super().__init__('radar_visualizer')
        
        # --- PARAMETERS ---
        
        # Gradient Configuration (RCS in dB)
        # Values below 'water_limit' are pure Blue. Values above 'land_limit' are pure Green.
        self.declare_parameter('rcs_water_limit', -20.0)
        self.declare_parameter('rcs_land_limit', -6.0)
        
        # Near-field Clutter Filter (Blind Zone)
        # Points closer than 'min_distance_filter' (meters) are forced to be visualized as water.
        # This suppresses the strong reflection from the water surface directly under the drone.
        self.declare_parameter('min_distance_filter', 3.0) 

        self.declare_parameter('output_frame', 'ARS_548')

        # Retrieve and store values for performance
        self.min_db = self.get_parameter('rcs_water_limit').value
        self.max_db = self.get_parameter('rcs_land_limit').value
        self.min_dist = self.get_parameter('min_distance_filter').value

        self.output_frame = self.get_parameter('output_frame').value
        
        # --- LOGGING CONFIGURATION ---
        self.get_logger().info("--------------------------------------------------")
        self.get_logger().info(" RADAR VISUALIZATION NODE STARTED")
        self.get_logger().info(f" Mode: Gradient Visualization")
        self.get_logger().info(f"   - Water Limit (Blue): < {self.min_db} dB")
        self.get_logger().info(f"   - Land Limit (Green): > {self.max_db} dB")
        self.get_logger().info(f" Near-field Filter: < {self.min_dist} meters (Forced to Water/Blue)")
        self.get_logger().info("--------------------------------------------------")
        
        # QoS Settings: Best Effort is required for high-frequency sensor data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=10
        )

        # Subscribers and Publishers
        self.sub = self.create_subscription(PointCloud2, '/radar/PointCloudDetection', self.cb, qos)
        self.pub = self.create_publisher(PointCloud2, '/radar/classified_cloud', qos)

    def pack_rgb(self, r, g, b):
        """
        Bitwise packing of RGB values (0-255) into a single 32-bit float for PointCloud2.
        """
        rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
        return struct.unpack('f', struct.pack('I', rgb_int))[0]

    def get_color_gradient(self, val):
        """
        Calculates the RGB color based on the RCS intensity value using linear interpolation.
        Returns a packed float representing the color.
        """
        # Clamp values to the defined range
        if val < self.min_db: val = self.min_db
        if val > self.max_db: val = self.max_db
        
        # Calculate ratio (0.0 = Deep Water, 1.0 = Solid Land)
        ratio = (val - self.min_db) / (self.max_db - self.min_db)
        
        # Interpolate Colors
        # Red is unused (0). 
        # Green scales with intensity (Land).
        # Blue scales inversely (Water).
        r = 0
        g = 255 * ratio
        b = 150 * (1.0 - ratio) # Using a lighter blue (150) for better visibility
        
        return self.pack_rgb(r, g, b)

    def cb(self, msg):
        """
        Main callback: Unpacks raw data, calculates distance/color, and repacks into a new PointCloud.
        """
        try:
            new_msg = PointCloud2()
            
            # --- 1. HEADER & SYNCHRONIZATION ---
            new_msg.header = msg.header 
            # Override timestamp to current system time. 
            # Necessary for visualizing recorded bags with latency without TF errors.
            new_msg.header.stamp = self.get_clock().now().to_msg()
            new_msg.header.frame_id = self.output_frame
            
            # --- 2. POINT CLOUD STRUCTURE ---
            new_msg.height = msg.height
            new_msg.width = msg.width
            new_msg.is_bigendian = msg.is_bigendian
            new_msg.is_dense = msg.is_dense
            
            # Define fields: X, Y, Z, Intensity, RGB
            new_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1)
            ]
            
            # 5 floats (x,y,z,int,rgb) * 4 bytes = 20 bytes per point
            new_msg.point_step = 20
            new_msg.row_step = new_msg.point_step * msg.width
            
            new_data = bytearray()
            orig = bytearray(msg.data)
            
            # Offset for RCS value in the original message (assumed byte 20)
            RCS_OFFSET = 20 

            # --- 3. DATA PROCESSING LOOP ---
            for i in range(msg.width):
                s = i * msg.point_step
                
                # Unpack coordinates (X, Y, Z)
                x = struct.unpack('f', orig[s:s+4])[0]
                y = struct.unpack('f', orig[s+4:s+8])[0]
                z = struct.unpack('f', orig[s+8:s+12])[0]
                
                # Unpack Intensity (RCS)
                # Handling potential bad formatting in raw data
                try:
                    val = float(struct.unpack('b', orig[s+RCS_OFFSET:s+RCS_OFFSET+1])[0])
                except: 
                    val = -50.0 # Default low value on error
                
                # --- FILTERING LOGIC ---
                # Calculate Euclidean distance from sensor
                dist = math.sqrt(x*x + y*y + z*z)

                # Apply Blind Zone Filter:
                # If point is too close (specular reflection), force it to be "Deep Water" (-100 dB)
                if dist < self.min_dist:
                    val = -100.0
                
                # Generate Color
                color = self.get_color_gradient(val)
                
                # Append new point data structure
                new_data.extend(struct.pack('fffff', x, y, z, val, color))

            new_msg.data = bytes(new_data)
            self.pub.publish(new_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing PointCloud: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RadarVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Radar Visualizer Node.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()