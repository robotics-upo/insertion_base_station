#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

class GPSPublisher(Node):
    """
    A ROS 2 node that publishes static GPS data (NavSatFix) for testing and simulation.
    
    This node acts as a virtual GPS sensor, publishing a fixed coordinate set
    at a user-defined frequency. It is useful for testing localization stacks
    without hardware.
    """

    def __init__(self):
        super().__init__('gps_publisher')

        # --- PARAMETER CONFIGURATION ---
        # Define default values (can be overridden via launch files)
        self.declare_parameter('latitude', 39.794259)
        self.declare_parameter('longitude', -4.081345)
        self.declare_parameter('altitude', 10.0)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('rate', 20.0)  # Frequency in Hz
        self.declare_parameter('status', NavSatStatus.STATUS_FIX)
        self.declare_parameter('service', NavSatStatus.SERVICE_GPS)

        # Retrieve parameter values
        self.lat = self.get_parameter('latitude').value
        self.lon = self.get_parameter('longitude').value
        self.alt = self.get_parameter('altitude').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate = self.get_parameter('rate').value
        
        # Cast integer parameters explicitly to ensure type safety
        self.status_val = int(self.get_parameter('status').value)
        self.service_val = int(self.get_parameter('service').value)

        # --- PUBLISHER SETUP ---
        self.pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        
        # Prevent division by zero if rate is set to 0
        timer_period = 1.0 / max(1e-3, self.rate)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'GPS Publisher initialized. Broadcasting coordinates ({self.lat}, {self.lon}) '
            f'at {self.rate} Hz on frame "{self.frame_id}".'
        )

    def timer_callback(self):
        """
        Callback function executed at the defined rate to publish the GPS fix.
        """
        msg = NavSatFix()
        
        # Timestamp and Frame ID
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # GPS Status (Fix type and Service)
        msg.status.status = self.status_val
        msg.status.service = self.service_val

        # Geodetic Position
        msg.latitude = self.lat
        msg.longitude = self.lon
        msg.altitude = self.alt

        # Position Covariance
        # Defined as a row-major 3x3 matrix. 
        # COVARIANCE_TYPE_UNKNOWN implies the covariance matrix is empty or invalid.
        msg.position_covariance = [0.0] * 9
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down GPS Publisher Node.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()