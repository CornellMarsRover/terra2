import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import sys
sys.path.append('/usr/lib/python3/dist-packages/serial')
import serial
import time
import pynmea2  # Make sure this library is installed

START_LAT = 40.7128  # Example starting latitude
START_LON = -74.0060  # Example starting longitude
START_ALT = 10.0      # Example starting altitude in meters

class GPSDataSubscriber(Node):

    def __init__(self):
        super().__init__('gps_data_subscriber')
        self.publisher_ = self.create_publisher(NavSatFix, 'fake_navsatfix_data', 10)

    def read_gps_data(self):
        while rclpy.ok():
            navsat_msg = NavSatFix()
            navsat_msg.header.stamp = self.get_clock().now().to_msg()
            navsat_msg.header.frame_id = 'gps_frame'  # Change to your frame ID
            navsat_msg.latitude = START_LAT
            navsat_msg.longitude = START_LON
            navsat_msg.altitude = START_ALT
            self.publish_gps_data(navsat_msg)
        time.sleep(0.5)  # Adjust the frequency as needed

    def publish_gps_data(self, navsatfix_msg):
        self.get_logger().info('Publishing Fake NavSatFix data')
        self.publisher_.publish(navsatfix_msg)

def main(args=None):
    rclpy.init(args=args)
    gps_data_subscriber = GPSDataSubscriber()

    try:
        gps_data_subscriber.read_gps_data()
        rclpy.spin(gps_data_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        gps_data_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()