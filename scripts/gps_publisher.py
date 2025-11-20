#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

"""
Simple ROS2 Python node that publishes sensor_msgs/NavSatFix messages.
Place this file in your package's scripts/ and make it executable:
    chmod +x gps_publisher.py
Run with:
    ros2 run <your_package> gps_publisher.py
or use ros2 launch / ros2 params to change parameters.
"""


class GPSPublisher(Node):
        def __init__(self):
                super().__init__('gps_publisher')

                # Parameters (can be set via ros2 param set or launch)
                self.declare_parameter('latitude', 37.7749)   # default: San Francisco
                self.declare_parameter('longitude', -122.4194)
                self.declare_parameter('altitude', 10.0)
                self.declare_parameter('frame_id', 'gps')
                self.declare_parameter('rate', 1.0)  # Hz
                self.declare_parameter('status', NavSatStatus.STATUS_FIX)
                self.declare_parameter('service', NavSatStatus.SERVICE_GPS)

                self.lat = float(self.get_parameter('latitude').get_parameter_value().double_value)
                self.lon = float(self.get_parameter('longitude').get_parameter_value().double_value)
                self.alt = float(self.get_parameter('altitude').get_parameter_value().double_value)
                self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
                self.rate = float(self.get_parameter('rate').get_parameter_value().double_value)
                self.status_val = int(self.get_parameter('status').get_parameter_value().integer_value)
                self.service_val = int(self.get_parameter('service').get_parameter_value().integer_value)

                self.pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
                timer_period = 1.0 / max(1e-3, self.rate)
                self.timer = self.create_timer(timer_period, self.timer_callback)
                self.get_logger().info(f'Publishing NavSatFix to "gps/fix" at {self.rate} Hz')

        def timer_callback(self):
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id

                # Status
                msg.status.status = self.status_val
                msg.status.service = self.service_val

                # Position
                msg.latitude = self.lat
                msg.longitude = self.lon
                msg.altitude = self.alt

                # Defaults: unknown covariance
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
                node.get_logger().info('Shutting down gps_publisher')
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
        main()