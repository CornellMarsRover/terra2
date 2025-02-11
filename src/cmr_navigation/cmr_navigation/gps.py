import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import sys
sys.path.append('/usr/lib/python3/dist-packages/serial')
import serial
import time
import pynmea2  # Make sure this library is installed
import numpy as np  # For covariance calculation

class GPSDataSubscriber(Node):

    def __init__(self, gps_port, baud_rate):
        super().__init__('gps_data_subscriber')
        self.gps_port = gps_port
        self.baud_rate = baud_rate
        self.serial_port = serial.Serial(self.gps_port, self.baud_rate, timeout=1)
        self.publisher_ = self.create_publisher(NavSatFix, 'navsatfix_data', 10)
        self.UERE = 3.0  # User Equivalent Range Error in meters 
                         # (adjust from 1-5 based on conditions 1-optimal 5-worst)
        self.get_logger().info("GPS node initialized")

    def parse_nmea_sentence(self, sentence):
        if sentence.startswith('$GPGGA'):
            try:
                msg = pynmea2.parse(sentence)
                navsat_msg = NavSatFix()
                navsat_msg.header.stamp = self.get_clock().now().to_msg()
                navsat_msg.header.frame_id = 'gps_frame'
                
                # Convert latitude and longitude to decimal degrees
                navsat_msg.latitude = float(msg.latitude)
                navsat_msg.longitude = float(msg.longitude)
                navsat_msg.altitude = 0.0
                
                
                try:
                    # Extract HDOP value
                    hdop = float(msg.horizontal_dil)
                except pynmea2.ParseError as e:
                    hdop = 2.5

                # Compute covariance (neglect altitude terms)
                variance = (hdop * self.UERE) ** 2
                navsat_msg.position_covariance = [
                    variance, 0.0, 0.0,
                    0.0, variance, 0.0,
                    0.0, 0.0, 0.0  # Neglecting altitude covariance
                ]
                navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                # Don't publish invalid readings
                if navsat_msg.latitude == 0.0 and navsat_msg.longitude == 0.0:
                    return None
                
                return navsat_msg
            except pynmea2.ParseError as e:
                self.get_logger().error(f"NMEA Parse Error: {e}")
                return None
            except ValueError as e:
                self.get_logger().error(f"HDOP Extraction Error: {e}")
                return None
                
    def read_gps_data(self):
        while rclpy.ok():
            try:
                data = self.serial_port.readline().decode('ascii', errors='replace').strip()
                if data:
                    navsatfix_msg = self.parse_nmea_sentence(data)
                    if navsatfix_msg is not None:
                        self.publish_gps_data(navsatfix_msg)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")

    def publish_gps_data(self, navsatfix_msg):
        self.publisher_.publish(navsatfix_msg)


def main(args=None):
    rclpy.init(args=args)
    gps_port = '/dev/ttyUSB1'  # Replace with your GPS device port
    baud_rate = 4800  # Common baud rate for GPS
    gps_data_subscriber = GPSDataSubscriber(gps_port, baud_rate)

    try:
        gps_data_subscriber.read_gps_data()
        rclpy.spin(gps_data_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        gps_data_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
