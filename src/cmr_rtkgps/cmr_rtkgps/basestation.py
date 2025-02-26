#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import socket
from pyubx2 import UBXReader

class GPSBasestation(Node):
    def __init__(self):
        super().__init__('gps_basestation')
        # Open serial port for RTK GPS module
        try:
            self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
            self.ubr = UBXReader(self.ser)
            self.get_logger().info("Serial port /dev/ttyACM0 opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
        
        # Set up UDP socket for broadcasting correction data
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Use the broadcast address; adjust as needed for your network
        self.broadcast_address = ('10.49.15.204', 4990)
        
        # Timer to poll the GPS module for correction data
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("GPS Basestation node started.")

    def timer_callback(self):
        try:
            (raw_data, parsed_data) = self.ubr.read()
            if raw_data:
                # Publish (broadcast) the raw correction data over UDP
                self.sock.sendto(raw_data, self.broadcast_address)
                self.get_logger().info(f"Sent correction data: {raw_data.hex()}")
        except Exception as e:
            self.get_logger().error(f"Error reading GPS data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSBasestation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
