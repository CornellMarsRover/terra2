import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import threading
import csv
import os
import sys
from datetime import datetime

class GPSLoggerNode(Node):
    def __init__(self):
        super().__init__('gps_logger_node')

        self.subscription = self.create_subscription(
            NavSatFix,
            '/rtk/navsatfix_data',
            self.listener_callback,
            10
        )

        self.latest_fix = None
        self.logged_positions = []
        self.shutdown_event = threading.Event()

        # Start the input thread
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        # Register shutdown handler
        self.get_logger().info('GPS Logger Node started. Press Enter to record current GPS location.')

    def listener_callback(self, msg):
        self.latest_fix = msg

    def wait_for_input(self):
        try:
            while not self.shutdown_event.is_set():
                input()  # Waits for Enter key
                if self.latest_fix:
                    lat = self.latest_fix.latitude
                    lon = self.latest_fix.longitude
                    self.logged_positions.append((lat, lon))
                    self.get_logger().info(f'Saved GPS point: ({lat}, {lon})')
                else:
                    self.get_logger().warn('No GPS data received yet.')
        except KeyboardInterrupt:
            pass  # Allow main thread to handle shutdown

    def save_to_csv(self):
        home = os.path.expanduser("~")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(home, f'gps_log_{timestamp}.csv')
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Latitude', 'Longitude'])
            writer.writerows(self.logged_positions)
        self.get_logger().info(f'Saved {len(self.logged_positions)} positions to {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = GPSLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user (Ctrl+C)')
    finally:
        node.shutdown_event.set()
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
