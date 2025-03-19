#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import socket
import threading
from pyubx2 import UBXReader, UBXMessage
import time

class GPSBasestation(Node):
    def __init__(self):
        super().__init__('gps_basestation')
        self.fixed_mode = False  # Flag to indicate if fixed mode has been set
        self.rover_ready = False # Flag to indicate if rover gps node started
        self.ready_sub = self.create_subscription(
            String,
            '/rtk/rover_ready',
            self.ready_callback,
            10
        )
        # ------------------------
        # 1. Open serial port
        # ------------------------
        try:
            self.ser = serial.Serial('/dev/ttyACM0', baudrate=230400, timeout=1)
            # Allow all protocol messages (UBX, RTCM, NMEA) to be decoded
            self.ubr = UBXReader(self.ser)
            self.get_logger().info("Serial port /dev/ttyACM0 opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # ------------------------
        # 2. Configure the ZED-F9P for Survey-In mode
        # ------------------------
        accuracy = 10
        self.accuracy_limit = accuracy*10
        self.survey_base_station()

        # ------------------------
        # 3. Set up UDP socket for broadcast
        # ------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Adjust broadcast address/port to match your network
        self.broadcast_address = ('10.49.15.204', 4990)

        # ------------------------
        # 4. Start reading and processing GPS messages in a background thread
        # ------------------------
        self.read_thread = threading.Thread(target=self.read_gps_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info("GPS Basestation node started in Survey-In mode.")
    
    def ready_callback(self, msg):
        if msg.data == 'ready':
            self.rover_ready = True

    def survey_base_station(self):
        """
        Configure the ZED-F9P for Survey-In mode:
          - Set Survey-In mode (CFG_TMODE_MODE = 1)
          - Set survey duration and accuracy limit
          - Enable UBX output over USB
        """
        transaction = 0 
        layers = 1  # Configure in volatile RAM for immediate effect
        clear_keys = ["CFG_UART1_BAUDRATE", "CFG_TMODE_MODE", "CFG_TMODE_POS_TYPE",]
        msg = UBXMessage.config_del(layers, transaction, clear_keys)
        self.ser.write(msg.serialize())

        cfgData = []
        cfgData.append(("CFG_UART1_BAUDRATE", 230400))
        cfgData.append(("CFG_TMODE_POS_TYPE", 0))       # 1 = LLH
        msg = UBXMessage.config_set(layers, transaction, cfgData)
        self.ser.write(msg.serialize())

        time.sleep(2)
        # (A) Survey-In mode settings
        cfgData.append(("CFG_TMODE_MODE", 1))                # 1 = Survey-In
        cfgData.append(("CFG_TMODE_SVIN_MIN_DUR", 60))         # Minimum survey duration in seconds
        cfgData.append(("CFG_TMODE_SVIN_ACC_LIMIT", self.accuracy_limit))     # Accuracy limit (1000 * 0.1 mm = 10cm)
        cfgData.append(("CFG_RATE_MEAS", 200))  # 5 Hz (200 ms)

        # (B) Enable NAV-SVIN output on USB
        cfgData.append(("CFG_USBOUTPROT_UBX", 1))
        cfgData.append(("CFG_USBOUTPROT_NMEA", 1))
        cfgData.append(("CFG_USBOUTPROT_RTCM3X", 0))
        cfgData.append(("CFG_MSGOUT_UBX_NAV_SVIN_USB", 1))

        msg = UBXMessage.config_set(layers, transaction, cfgData)
        self.ser.write(msg.serialize())
        self.get_logger().info("Configured Survey-In mode and enabled RTCM output.")


    def configure_fixed_mode(self, svin_msg):
        """
        Once survey-in conditions are met, reconfigure the module to Fixed mode using the
        coordinates obtained from the survey.
        """
        transaction = 0
        layers = 1  # Use volatile RAM for immediate effect
        cfgData = []
        # (A) Switch to Fixed mode
        cfgData.append(("CFG_TMODE_MODE", 2))         # 2 = Fixed mode
        cfgData.append(("CFG_TMODE_POS_TYPE", 0))       # 0 = ECEF

        # (B) Set the base station's position using the survey results.
        #     The survey message (NAV-SVIN) provides the ECEF coords.
        cfgData.append(("CFG_TMODE_ECEF_X", svin_msg.meanX))
        cfgData.append(("CFG_TMODE_ECEF_Y", svin_msg.meanY))
        cfgData.append(("CFG_TMODE_ECEF_Z", svin_msg.meanZ))

        cfgData.append(("CFG_USBOUTPROT_UBX", 0))
        cfgData.append(("CFG_USBOUTPROT_NMEA", 0))
        cfgData.append(("CFG_USBOUTPROT_RTCM3X", 1))           # Enable RTCM output

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
                               f"meanX: {svin_msg.meanX}, Lon: {svin_msg.meanY}, Height: {svin_msg.meanZ}")
        self.fixed_mode = True

    def read_gps_loop(self):
        """
        Continuously read from the ZED-F9P. While in Survey-In mode, display survey info
        and check if the accuracy criteria are met. Once fixed mode is enabled, forward RTCM
        messages over UDP.
        """
        while rclpy.ok():
            try:
                msg = self.ubr.read()
                #self.get_logger().info(f"{msg}")
                if msg[0] is None:
                    #self.survey_base_station()
                    continue
                raw, parsed = msg[0], msg[1]
                self.get_logger().info(f"{parsed}")
                # Check for survey-in status messages (NAV-SVIN)
                if not self.fixed_mode and parsed.identity == "NAV-SVIN":
                    self.get_logger().info(
                        f"Survey-In: Duration = {parsed.dur}s, Accuracy = {parsed.meanAcc / 1e4}m"
                    )
                    # If both the survey duration and accuracy criteria are met, switch to Fixed mode.
                    if (parsed.meanAcc <= self.accuracy_limit) and (parsed.dur >= 60):
                        if not self.fixed_mode:
                            self.get_logger().info("Survey complete. Switching to Fixed mode...")
                            self.configure_fixed_mode(parsed)
                # Once in Fixed mode, forward RTCM messages (e.g., RTCM 1005, etc.)
                elif self.fixed_mode and self.rover_ready and parsed.identity.startswith("RTCM"):
                    self.sock.sendto(raw, self.broadcast_address)
                    self.get_logger().info("Broadcasting RTCM Correction")
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
