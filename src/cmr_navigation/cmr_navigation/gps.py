import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import sys
sys.path.append('/usr/lib/python3/dist-packages/serial')
import serial
import time
import pynmea2  # Make sure this library is installed

class GPSDataSubscriber(Node):

    def __init__(self, gps_port, baud_rate):
        super().__init__('gps_data_subscriber')
        self.gps_port = gps_port
        self.baud_rate = baud_rate
        self.serial_port = serial.Serial(self.gps_port, self.baud_rate, timeout=1)
        self.publisher_ = self.create_publisher(NavSatFix, 'navsatfix_data', 10)

    def parse_nmea_sentence(self, sentence):
        if sentence.startswith('$GPGGA'):
            try:
                msg = pynmea2.parse(sentence)
                navsat_msg = NavSatFix()
                navsat_msg.header.stamp = self.get_clock().now().to_msg()
                navsat_msg.header.frame_id = 'gps_frame'  # Change to your frame ID

                # Convert latitude and longitude to decimal degrees
                navsat_msg.latitude = float(msg.latitude)
                navsat_msg.longitude = float(msg.longitude)
                navsat_msg.altitude = float(msg.altitude)
           

                return navsat_msg
            except pynmea2.ParseError as e:
                self.get_logger().error(f"NMEA Parse Error: {e}")
                return None
    def read_gps_data(self):
        while rclpy.ok():
            try:
                data = self.serial_port.readline().decode('ascii', errors='replace').strip()
                if data:
                    navsatfix_msg = self.parse_nmea_sentence(data)
                    if navsatfix_msg is not None:  # Check if the message is not None
                        self.publish_gps_data(navsatfix_msg)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
            time.sleep(0.5)  # Adjust the frequency as needed

    def publish_gps_data(self, navsatfix_msg):
        self.get_logger().info('Publishing NavSatFix data')
        self.publisher_.publish(navsatfix_msg)

def main(args=None):
    rclpy.init(args=args)
    gps_port = '/dev/ttyUSB0'  # replace with your GPS device port
    baud_rate = 4800  # common baud rate for GPS
    gps_data_subscriber = GPSDataSubscriber(gps_port, baud_rate)

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
