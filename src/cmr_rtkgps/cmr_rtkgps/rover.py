#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import socket
import threading
from pyubx2 import UBXReader, UBXMessage

class GPSRover(Node):
    def __init__(self):
        super().__init__('gps_rover')

        # ------------------------
        # 1. Open local serial port for the rover’s ZED-F9P
        # ------------------------
        try:
            self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
            # We want to parse UBX, possibly also see if RTCM is recognized. 
            # ubxonly=False so that RTCM is recognized if it appears in the stream.
            self.ubr = UBXReader(self.ser, ubxonly=False)  
            self.get_logger().info("Rover serial port /dev/ttyACM0 opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # ------------------------
        # 2. Configure local ZED-F9P for RTK rover:
        #    a) Enable RTCM input over USB
        #    b) Output UBX-NAV-PVT so we can read lat/lon
        # ------------------------
        self.configure_rover()

        # ------------------------
        # 3. Set up UDP socket to receive RTCM corrections
        # ------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 4990))

        # Use a lock for the incoming RTCM data
        self.latest_correction = None
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

        SET = "SET"
        cfgData = []

        # (A) Enable RTCM input on USB
        cfgData.append(("CFG-USBINPROT-RTCM3X", 1))   # 1 = enable

        # (B) Enable UBX output on USB, so we get NAV-PVT
        cfgData.append(("CFG-USBOUTPROT-UBX", 1))

        # (C) Make sure we are outputting NAV-PVT at 1 Hz:
        # CFG-MSGOUT-UBX_NAV_PVT_USB
        cfgData.append(("CFG-MSGOUT-UBX_NAV_PVT_USB", 1))

        # Build the config message
        msg = UBXMessage.config_set(
            cfgData,
            layer=1  # set to RAM only for demonstration
        )
        # Send
        self.ser.write(msg.serialize())
        self.get_logger().info("Configured rover for RTCM input + UBX NAV-PVT output.")

    def listen_for_corrections(self):
        """
        Continuously listen for RTCM data from the basestation over UDP
        and store the latest chunk in self.latest_correction.
        """
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(2048)  # Might need bigger than 1024
                with self.lock:
                    self.latest_correction = data
                self.get_logger().debug(f"Received {len(data)} bytes of RTCM from {addr}")
            except Exception as e:
                self.get_logger().error(f"Error receiving UDP data: {e}")

    def timer_callback(self):
        """
        Periodic tasks (10 Hz):
          - If new RTCM corrections arrived, write them to the local GNSS module.
          - Read any new UBX messages (especially NAV-PVT) and log the position.
        """
        # 1) Write any newly arrived RTCM corrections
        with self.lock:
            if self.latest_correction:
                try:
                    self.ser.write(self.latest_correction)
                    self.latest_correction = None
                except Exception as e:
                    self.get_logger().error(f"Error writing RTCM data to GNSS: {e}")

        # 2) Read any local messages from the ZED-F9P
        try:
            (parsed_data, raw_data) = self.ubr.read()
            if parsed_data:
                # If it is a NAV-PVT message, extract lat/lon
                if parsed_data.identity == "NAV-PVT":
                    # lat, lon are in degrees * 1e-7
                    lat_deg = parsed_data.lat / 1e7
                    lon_deg = parsed_data.lon / 1e7
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
