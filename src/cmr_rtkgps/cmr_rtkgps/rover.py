#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

import serial
import socket
import threading
from pyubx2 import UBXReader, UBXMessage, llh2ecef
import time

class GPSRover(Node):
    def __init__(self):
        super().__init__('gps_rover')

        # Navsatfix data publisher
        self.pub = self.create_publisher(NavSatFix, '/rtk/navsatfix_data', 10)

        # ------------------------
        # 1. Open local serial port for the rover’s ZED-F9P
        # ------------------------
        try:
            self.ser = serial.Serial('/dev/ttyACM0', baudrate=230400, timeout=1)
            # We want to parse UBX, possibly also see if RTCM is recognized. 
            # ubxonly=False so that RTCM is recognized if it appears in the stream.
            self.ubr = UBXReader(self.ser)  
            self.get_logger().info("Rover serial port /dev/ttyACM0 opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # ------------------------
        # 2. Configure local ZED-F9P for RTK rover:
        #    a) Enable RTCM input over USB
        #    b) Output UBX-NAV-PVT so we can read lat/lon
        # ------------------------
        # LLH coordinates (lat/lon in decimal, height in meters) - 'LAT', 'LON', 'ALT'
        # ECEF coordinates (all in meters) - 'X', 'Y', 'Z' 
        self.fix = {
            'LLH': True,  # Boolean to indicate if using LLH or ECEF
            'LAT': 42.443962,  # decimals
            'LON': -76.482985,  # decimals
            'ALT': 240.0,      # meters
        }
        self.north_offset = 0.0 # north offset from known start in meters
        self.east_offset = 0.0 # east offset from known start in meters
        self.configure_rover()

        # ------------------------
        # 3. Set up TCP socket to receive RTCM corrections
        # ------------------------
        self.server_ip = '10.49.49.190'  # Basestation IP
        self.server_port = 4990          # Same port as the basestation
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.sock.connect((self.server_ip, self.server_port))
            self.get_logger().info(f"Connected to basestation at {self.server_ip}:{self.server_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to basestation: {e}")
            return

        # Use a lock for the incoming RTCM data
        self.lock = threading.Lock()

        # Start a background thread to listen for corrections
        self.rx_thread = threading.Thread(target=self.listen_for_corrections, daemon=True)
        self.rx_thread.start()

        # ------------------------
        # 4. Start a timer (10 Hz) to:
        #    - Feed any new RTCM data to the local receiver
        #    - Read UBX messages (NAV-PVT) to display current position
        # ------------------------
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("GPS Rover node started.")

    def configure_rover(self):
        """
        Configure the local ZED-F9P to accept RTCM input on USB
        and output UBX-NAV-PVT for position.
        """
        # We'll do CFG-VALSET on the rover:
        # 1) Enable RTCM input on USB
        # 2) Enable UBX output of NAV-PVT at 1 Hz (or more).
        # 3) Rover is by default in 'RTK Fix' mode if corrections are valid 
        #    (CFG-NAVHPG-DGNSSMODE=2). That is usually the default.

        transaction = 0 
        layers = 1 # RAM
        cfgData = []
        cfgData.append(("CFG_UART1_BAUDRATE", 230400))
        # (A) Switch to Fixed mode
        cfgData.append(("CFG_TMODE_MODE", 0))         # 0 = Rover mode
        cfgData.append(("CFG_TMODE_POS_TYPE", 0))       # 0 = LLH
        '''msg = UBXMessage.config_set(layers, transaction, cfgData)
        self.ser.write(msg.serialize())
        cfgData = []

        # (B) Set initial fix position
        lat, lon, h = self.fix['LAT'], self.fix['LON'], self.fix['ALT']
        x, y, z = llh2ecef(lat, lon, h)
        x = int((x+self.north_offset)*100)
        y = int((y+self.east_offset)*100)
        z = int(z*100)
        self.get_logger().info(f"{x}  {y}   {z}")
        cfgData.append(("CFG_TMODE_ECEF_X", x))
        cfgData.append(("CFG_TMODE_ECEF_Y", y))
        cfgData.append(("CFG_TMODE_ECEF_Z", z))'''

        # (C) Enable RTCM input on USB
        cfgData.append(("CFG_USBINPROT_RTCM3X", 1))   # 1 = enable

        # (D) Enable UBX output on USB, so we get NAV_PVT
        cfgData.append(("CFG_USBOUTPROT_UBX", 1))
        cfgData.append(("CFG_USBOUTPROT_NMEA", 0))
        cfgData.append(("CFG_USBOUTPROT_RTCM3X", 0))

        # (E) Make sure we are outputting NAV_PVT at 1 Hz:
        # CFG_MSGOUT_UBX_NAV_PVT_USB
        #cfgData.append(("CFG_MSGOUT_UBX_NAV_PVT_USB", 1))
        cfgData.append(("CFG_RATE_MEAS", 200))  # 5 Hz (200 ms)

        # Build the config message
        msg = UBXMessage.config_set(layers, transaction, cfgData)
        # Send
        self.ser.write(msg.serialize())
        self.get_logger().info("Configured rover for RTCM input + UBX NAV_PVT output.")

    def listen_for_corrections(self):
        """
        Continuously listen for RTCM data from the basestation over UDP
        and store the latest chunk in self.latest_correction.
        """
        while rclpy.ok():
            try:
                data = self.sock.recv(2048)
                with self.lock:
                    try:
                        self.ser.write(data)
                    except Exception as e:
                        self.get_logger().error(f"Error writing RTCM data to GNSS: {e}")
                self.get_logger().info("Received RTCM Correction")
            except Exception as e:
                self.get_logger().error(f"Error receiving UDP data: {e}")

    def timer_callback(self):
        """
        Periodic tasks (10 Hz):
          - Read any new UBX messages and log the position.
        """

        # Read any local messages from the ZED-F9P
        try:
            (raw_data, parsed_data) = self.ubr.read()
            if parsed_data:
                #self.get_logger().info(f"{parsed_data}")
                # lat, lon are in degrees * 1e-7
                lat_deg = parsed_data.lat
                lon_deg = parsed_data.lon
                
                msg = NavSatFix()
                msg.latitude = lat_deg
                msg.longitude = lon_deg
                msg.header.stamp = self.get_clock().now().to_msg()

                self.pub.publish(msg)
                # Print or log the position
                self.get_logger().info(f"Rover: lat={lat_deg:.7f}, lon={lon_deg:.7f}, hAcc={parsed_data.hAcc} mm")
        except Exception as e:
            self.get_logger().error(f"Error reading local GPS data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSRover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
