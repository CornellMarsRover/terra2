import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import threading
import csv
import os
import sys
from datetime import datetime

class DisplacementLoggerNode(Node):
    def __init__(self):
        super().__init__('displacement_logger_node')

        self.subscription = self.create_subscription(
            TwistStamped,
            '/autonomy/pose/global/robot',
            self.listener_callback,
            10
        )

        self.latest_twist = None
        self.logged_displacements = []
        self.shutdown_event = threading.Event()

        # Start the input thread
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info("Displacement Logger Node started. Press Enter to record displacement.")

    def listener_callback(self, msg: TwistStamped):
        self.latest_twist = msg.twist.linear

    def wait_for_input(self):
        try:
            while not self.shutdown_event.is_set():
                input()  # Waits for Enter key
                if self.latest_twist:
                    north = self.latest_twist.x
                    west = self.latest_twist.y
                    self.logged_displacements.append((north, west))
                    self.get_logger().info(f"Saved displacement: North = {north} m, West = {west} m")
                else:
                    self.get_logger().warn("No TwistStamped data received yet.")
        except KeyboardInterrupt:
            pass  # Main thread handles shutdown

    def save_to_csv(self):
        home = os.path.expanduser("~")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(home, f'displacement_log_{timestamp}.csv')
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['displacement north (meters)', 'displacement west (meters)'])
            writer.writerows(self.logged_displacements)
        self.get_logger().info(f"Saved {len(self.logged_displacements)} displacements to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = DisplacementLoggerNode()

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
