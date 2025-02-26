#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import socket
import threading
from pyubx2 import UBXReader, UBXMessage

class GPSBasestation(Node):
    def __init__(self):
        super().__init__('gps_basestation')

        # ------------------------
        # 1. Open serial port
        # ------------------------
        try:
            self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
            self.ubr = UBXReader(self.ser, ubxonly=False) 
            # ubxonly=False allows RTCM messages to be identified as well
            self.get_logger().info("Serial port /dev/ttyACM0 opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # ------------------------
        # 2. Configure the ZED-F9P for Survey-In RTK base
        #    a) Enable Survey-In
        #    b) Enable RTCM output messages
        # ------------------------
        self.configure_base_station()

        # ------------------------
        # 3. Set up UDP socket for broadcast
        # ------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Adjust broadcast address/port to match your network
        self.broadcast_address = ('10.49.15.255', 4990)

        # ------------------------
        # 4. Start reading & sending RTCM in a background thread
        # ------------------------
        self.read_thread = threading.Thread(target=self.read_gps_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info("GPS Basestation node started.")

    def configure_base_station(self):
        """
        Configure the local ZED-F9P for:
          - Survey-In Mode
          - Output recommended RTCM messages over USB
        """
        # NOTE: If your device is truly on UART1, replace the USB references with UART1 equivalents.
        # We are using CFG-VALSET with transaction = "SET". You can either keep them in volatile RAM
        # or save to flash (layer=7). For demonstration, we do layer=1 (RAM).
        SET = "SET"
        cfgData = []
        
        # (A) Enable Survey-In Mode: CFG-TMODE-MODE = 1 (Survey-In)
        #     Also set minimum duration, accuracy limit, etc.
        cfgData.append(("CFG-TMODE-MODE", 1))                # 1 = Survey-In, 2 = Fixed, 0 = Disabled
        cfgData.append(("CFG-TMODE-SVIN_MIN_DUR", 60))       # Minimum survey in time (seconds). Adjust to suit.
        cfgData.append(("CFG-TMODE-SVIN_ACC_LIMIT", 100000)) # Position accuracy limit (0.1 m = 100000 = 1e5 units in mm)

        # (B) Enable RTCM output on USB
        cfgData.append(("CFG-USBOUTPROT-RTCM3X", 1))  # Turn on RTCM output over USB

        # (C) Enable the recommended RTCM messages at e.g. 1 Hz
        #     1005, 1074, 1084, 1094, 1124, 1230
        #     Each key is: CFG-MSGOUT-RTCM_3X_TYPExxxx_USB
        #     Setting the value to 1 means 1 message per navigation cycle (by default 1 Hz).
        cfgData.append(("CFG-MSGOUT-RTCM_3X_TYPE1005_USB", 1))
        cfgData.append(("CFG-MSGOUT-RTCM_3X_TYPE1074_USB", 1))
        cfgData.append(("CFG-MSGOUT-RTCM_3X_TYPE1084_USB", 1))
        cfgData.append(("CFG-MSGOUT-RTCM_3X_TYPE1094_USB", 1))
        cfgData.append(("CFG-MSGOUT-RTCM_3X_TYPE1124_USB", 1))
        cfgData.append(("CFG-MSGOUT-RTCM_3X_TYPE1230_USB", 1))

        # Build the UBX message. 
        # layer 1 = RAM, layer 2 = BBR, layer 4 = Flash. 7 = BBR+Flash+RAM. 
        # For demonstration, set to layer=1 (RAM) so these take effect immediately
        msg = UBXMessage.config_set(
            cfgData,
            layer=2  # 2 = BBR
        )
        
        # Send message to the receiver
        self.ser.write(msg.serialize())
        self.get_logger().info("Sent Survey-In + RTCM output config to the ZED-F9P.")

    def read_gps_loop(self):
        """
        Continuously read from the ZED-F9P, filter for RTCM messages,
        and broadcast them over UDP for the rover.
        """
        while rclpy.ok():
            try:
                # read() returns a tuple (parsed_data, raw_data) by default
                parsed_data, raw_data = self.ubr.read()
                if parsed_data:
                    # If it's an RTCM message, broadcast it
                    if parsed_data.identity.startswith("RTCM"):
                        self.sock.sendto(raw_data, self.broadcast_address)
                        self.get_logger().debug(
                            f"Broadcasting RTCM ({parsed_data.identity}), len={len(raw_data)} bytes."
                        )
            except Exception as e:
                self.get_logger().error(f"Error reading/streaming GPS data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSBasestation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
