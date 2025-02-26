#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import socket
import threading
from pyubx2 import UBXReader

class GPSRover(Node):
    def __init__(self):
        super().__init__('gps_rover')
        # Open serial port for local RTK GPS module
        try:
            self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
            self.ubr = UBXReader(self.ser)
            self.get_logger().info("Serial port /dev/ttyACM0 opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
        
        # Set up UDP socket to receive correction data
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 4990))
        self.latest_correction = None
        self.lock = threading.Lock()
        
        # Start a thread to listen for incoming correction data
        threading.Thread(target=self.listen_for_corrections, daemon=True).start()
        
        # Timer to poll the local GPS module for data
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("GPS Rover node started.")

    def listen_for_corrections(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)  # buffer size in bytes
                with self.lock:
                    self.latest_correction = data
                self.get_logger().info(f"Received correction data from {addr}: {data.hex()}")
            except Exception as e:
                self.get_logger().error(f"Error receiving UDP data: {e}")

    def timer_callback(self):
        try:
            (raw_data, parsed_data) = self.ubr.read()
            if raw_data:
                with self.lock:
                    correction = self.latest_correction
                # Here, you would normally apply the correction to your raw GPS data.
                # For demonstration, we simply combine the parsed GPS data with the latest correction.
                corrected_reading = self.apply_correction(parsed_data, correction)
                self.get_logger().info(f"Corrected GPS reading: {corrected_reading}")
        except Exception as e:
            self.get_logger().error(f"Error reading local GPS data: {e}")

    def apply_correction(self, gps_data, correction_data):
        # Replace this stub with actual RTK correction logic as needed.
        if correction_data:
            return f"{gps_data} with correction {correction_data.hex()}"
        else:
            return f"{gps_data} (no correction available)"

def main(args=None):
    rclpy.init(args=args)
    node = GPSRover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
