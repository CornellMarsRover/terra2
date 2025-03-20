#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import socket
import threading
from pyubx2 import UBXReader, UBXMessage
from pyubx2 import llh2ecef, ecef2llh
import time

class GPSBasestation(Node):
    def __init__(self):

        super().__init__('gps_basestation')

        # ------------------------
        # 1. Open serial port
        # ------------------------
        try:
            self.ser = serial.Serial('/dev/ttyACM0', baudrate=230400, timeout=1)
            self.ubr = UBXReader(self.ser)
            self.get_logger().info("Serial port /dev/ttyACM0 opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return
        
        # ------------------------
        # 2. Configure fixed mode for basestation
        # ------------------------
        # LLH coordinates (lat/lon in decimal, height in meters) - 'LAT', 'LON', 'ALT'
        self.fix = {
            'LAT': 42.4449177,  # decimals
            'LON': -76.4836309,  # decimals
            'ALT': 245.0,      # meters
            'x': 110183650,
            'y': -458341884,
            'z': 428228837
        }
        self.configure_fixed_mode()

        # ------------------------
        # 3. Set up TCP socket for broadcast
        # ------------------------
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_address = ('0.0.0.0', 4990)  # All IPs
        self.server_socket.bind(self.server_address)
        self.server_socket.listen(1)  # Only one connection (rover)
        self.get_logger().info(f"Waiting for rover to connect on {self.server_address}...")

        # Accept connection from rover (this blocks until rover connects)
        self.client_socket, self.client_address = self.server_socket.accept()
        self.get_logger().info(f"Rover connected from {self.client_address}")
        
        # ------------------------
        # 4. Start reading and processing GPS messages in a background thread
        # ------------------------
        self.read_thread = threading.Thread(target=self.read_gps_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info("GPS Basestation node started in Survey-In mode.")

    def configure_fixed_mode(self):
        """
        Configure the module to Fixed mode using the known coordinates
        """
        transaction = 0
        layers = 1  # Use volatile RAM for immediate effect
        cfgData = []
        cfgData.append(("CFG_UART1_BAUDRATE", 230400))

        # (A) Switch to Fixed mode
        cfgData.append(("CFG_TMODE_MODE", 2))         # 2 = Fixed mode
        cfgData.append(("CFG_TMODE_POS_TYPE", 0))       # 0 = LLH (necessary for RTCM 1005)
        msg = UBXMessage.config_set(layers, transaction, cfgData)
        self.ser.write(msg.serialize())

        cfgData = []
        # (B) Set the base station's position using the known starting point
        '''lat, lon, h = self.fix['LAT'], self.fix['LON'], self.fix['ALT']
        x, y, z = llh2ecef(lat, lon, h)
        x = int(x*100)
        y = int(y*100)
        z = int(z*100)
        self.get_logger().info(f"ECEF coordinates: {x}  {y}  {z}")'''
        x, y, z = self.fix['x'], self.fix['y'], self.fix['z']
        cfgData.append(("CFG_TMODE_ECEF_X", x))
        cfgData.append(("CFG_TMODE_ECEF_Y", y))
        cfgData.append(("CFG_TMODE_ECEF_Z", z))

        cfgData.append(("CFG_USBOUTPROT_UBX", 0))
        cfgData.append(("CFG_USBOUTPROT_NMEA", 0))
        cfgData.append(("CFG_USBOUTPROT_RTCM3X", 1))

        # (C) Enable the recommended RTCM messages (output at 1 Hz)
        cfgData.append(("CFG_MSGOUT_RTCM_3X_TYPE1005_USB", 1))
        cfgData.append(("CFG_MSGOUT_RTCM_3X_TYPE1074_USB", 1))
        cfgData.append(("CFG_MSGOUT_RTCM_3X_TYPE1084_USB", 1))
        cfgData.append(("CFG_MSGOUT_RTCM_3X_TYPE1094_USB", 1))
        cfgData.append(("CFG_MSGOUT_RTCM_3X_TYPE1124_USB", 1))
        cfgData.append(("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 1))

        msg = UBXMessage.config_set(layers, transaction, cfgData)
        self.ser.write(msg.serialize())
        self.get_logger().info(f"Configured Fixed mode with coordinates: "
                               f"Lat: {lat}, Lon: {lon}, Height: {h}m"
                               f"ECEF: ({x}m, {y}m, {z}m)")

    def read_gps_loop(self):
        """
        Continuously read from the ZED-F9P. While in Survey-In mode, display survey info
        and check if the accuracy criteria are met. Once fixed mode is enabled, forward RTCM
        messages over UDP.
        """
        while rclpy.ok():
            try:
                msg = self.ubr.read()
                if msg[0] is None:
                    continue
                raw, parsed = msg[0], msg[1]
                self.get_logger().info(f"{parsed}")
                self.get_logger().info(f"{parsed.identity}")
                self.client_socket.sendall(raw)
                self.get_logger().info("Broadcasting RTCM correction")
            except Exception as e:
                self.get_logger().error(f"Error processing GPS data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSBasestation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
