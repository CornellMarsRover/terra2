#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import socket
import threading
import tkinter as tk
from tkinter import ttk
from pyubx2 import UBXReader, UBXMessage
from pyubx2 import ecef2llh
import time

class GPSBasestation(Node):
    def __init__(self):

        super().__init__('gps_basestation')
        self.started = False
        self.ser = None
        self.server_socket = None
        self.client_socket = None

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
        # 2. Get live base station coordinates and configure fixed mode
        # ------------------------
        self.fix = None
        self.select_basestation_fix()
        if self.fix is None:
            self.get_logger().error("No base station coordinates were selected.")
            return
        self.configure_fixed_mode()

        # ------------------------
        # 3. Set up TCP socket for broadcast
        # ------------------------
        if not self.setup_server_socket():
            self.close_resources()
            return

        self.get_logger().info(f"Waiting for rover to connect on {self.server_address}...")

        # Accept connection from rover (this blocks until rover connects)
        try:
            self.client_socket, self.client_address = self.server_socket.accept()
        except Exception as e:
            self.get_logger().error(f"Failed while waiting for rover connection: {e}")
            self.close_resources()
            return
        self.get_logger().info(f"Rover connected from {self.client_address}")
        
        # ------------------------
        # 4. Start reading and processing GPS messages in a background thread
        # ------------------------
        self.read_thread = threading.Thread(target=self.read_gps_loop, daemon=True)
        self.read_thread.start()

        self.started = True
        self.get_logger().info("GPS Basestation node started in Fixed mode.")

    def setup_server_socket(self):
        self.server_address = ('0.0.0.0', 4990)  # All IPs
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.server_socket.bind(self.server_address)
            self.server_socket.listen(1)  # Only one connection (rover)
        except OSError as e:
            self.get_logger().error(
                f"Could not open basestation TCP server on {self.server_address}: {e}. "
                "Stop any old basestation/rover process using port 4990, then launch again."
            )
            return False

        return True

    def close_resources(self):
        for resource in (self.client_socket, self.server_socket, self.ser):
            if resource is None:
                continue
            try:
                resource.close()
            except Exception:
                pass

    def configure_position_selection_mode(self):
        """
        Configure the ZED-F9P to report live survey coordinates before the user
        chooses the fixed base station position.
        """
        transaction = 0
        layers = 1

        clear_keys = [
            "CFG_UART1_BAUDRATE",
            "CFG_TMODE_MODE",
            "CFG_TMODE_POS_TYPE",
        ]
        msg = UBXMessage.config_del(layers, transaction, clear_keys)
        self.ser.write(msg.serialize())

        cfgData = []
        cfgData.append(("CFG_UART1_BAUDRATE", 230400))
        cfgData.append(("CFG_TMODE_POS_TYPE", 0))
        msg = UBXMessage.config_set(layers, transaction, cfgData)
        self.ser.write(msg.serialize())

        time.sleep(2)

        cfgData.append(("CFG_TMODE_MODE", 1))
        cfgData.append(("CFG_TMODE_SVIN_MIN_DUR", 60))
        cfgData.append(("CFG_TMODE_SVIN_ACC_LIMIT", 100))
        cfgData.append(("CFG_RATE_MEAS", 200))
        cfgData.append(("CFG_USBOUTPROT_UBX", 1))
        cfgData.append(("CFG_USBOUTPROT_NMEA", 1))
        cfgData.append(("CFG_USBOUTPROT_RTCM3X", 0))
        cfgData.append(("CFG_MSGOUT_UBX_NAV_SVIN_USB", 1))

        msg = UBXMessage.config_set(layers, transaction, cfgData)
        self.ser.write(msg.serialize())
        self.get_logger().info("Showing live survey coordinates. Press Set to use the current fix.")

    def select_basestation_fix(self):
        """
        Show a small Tkinter window with live survey coordinates. The Set button
        stores the current coordinates and lets startup continue into fixed mode.
        """
        self.configure_position_selection_mode()
        latest_fix = {}
        latest_lock = threading.Lock()
        stop_reader = threading.Event()

        def read_live_coordinates():
            while not stop_reader.is_set() and rclpy.ok():
                try:
                    msg = self.ubr.read()
                    if msg[0] is None or msg[1] is None:
                        continue

                    parsed = msg[1]
                    if parsed.identity != "NAV-SVIN":
                        continue

                    x_cm = parsed.meanX
                    y_cm = parsed.meanY
                    z_cm = parsed.meanZ
                    lat, lon, alt = ecef2llh(x_cm / 100.0, y_cm / 100.0, z_cm / 100.0)
                    fix = {
                        'LAT': lat,
                        'LON': lon,
                        'ALT': alt,
                        'x': x_cm,
                        'y': y_cm,
                        'z': z_cm,
                        'dur': parsed.dur,
                        'acc': parsed.meanAcc / 1e4,
                    }
                    with latest_lock:
                        latest_fix.clear()
                        latest_fix.update(fix)
                except Exception as e:
                    if not stop_reader.is_set():
                        self.get_logger().error(f"Error reading live GPS coordinates: {e}")

        reader = threading.Thread(target=read_live_coordinates, daemon=True)
        reader.start()

        root = tk.Tk()
        root.title("Base Station Coordinates")
        root.resizable(False, False)

        fields = {
            "Latitude": tk.StringVar(value="Waiting for GPS..."),
            "Longitude": tk.StringVar(value="Waiting for GPS..."),
            "Altitude": tk.StringVar(value="Waiting for GPS..."),
            "ECEF X": tk.StringVar(value="Waiting for GPS..."),
            "ECEF Y": tk.StringVar(value="Waiting for GPS..."),
            "ECEF Z": tk.StringVar(value="Waiting for GPS..."),
            "Survey Time": tk.StringVar(value="Waiting for GPS..."),
            "Mean Accuracy": tk.StringVar(value="Waiting for GPS..."),
        }
        status = tk.StringVar(value="Waiting for live base station coordinates.")

        frame = ttk.Frame(root, padding=12)
        frame.grid(row=0, column=0, sticky="nsew")

        for row, (label, value) in enumerate(fields.items()):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", padx=(0, 12), pady=2)
            ttk.Label(frame, textvariable=value, width=28).grid(row=row, column=1, sticky="e", pady=2)

        ttk.Label(frame, textvariable=status).grid(
            row=len(fields),
            column=0,
            columnspan=2,
            sticky="w",
            pady=(10, 6),
        )

        set_button = ttk.Button(frame, text="Set", state=tk.DISABLED)
        set_button.grid(row=len(fields) + 1, column=1, sticky="e")

        def refresh_window():
            with latest_lock:
                fix = dict(latest_fix)

            if fix:
                fields["Latitude"].set(f"{fix['LAT']:.8f} deg")
                fields["Longitude"].set(f"{fix['LON']:.8f} deg")
                fields["Altitude"].set(f"{fix['ALT']:.3f} m")
                fields["ECEF X"].set(f"{fix['x']} cm")
                fields["ECEF Y"].set(f"{fix['y']} cm")
                fields["ECEF Z"].set(f"{fix['z']} cm")
                fields["Survey Time"].set(f"{fix['dur']} s")
                fields["Mean Accuracy"].set(f"{fix['acc']:.4f} m")
                status.set("Press Set to use the currently displayed coordinates.")
                set_button.configure(state=tk.NORMAL)

            root.after(250, refresh_window)

        def set_current_fix():
            with latest_lock:
                if not latest_fix:
                    return
                self.fix = dict(latest_fix)

            stop_reader.set()
            root.destroy()

        def cancel_selection():
            stop_reader.set()
            root.destroy()

        set_button.configure(command=set_current_fix)
        root.protocol("WM_DELETE_WINDOW", cancel_selection)
        refresh_window()
        root.mainloop()
        stop_reader.set()
        reader.join(timeout=2.0)

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
        cfgData.append(("CFG_TMODE_POS_TYPE", 0))       # 0 = ECEF
        msg = UBXMessage.config_set(layers, transaction, cfgData)
        self.ser.write(msg.serialize())

        cfgData = []
        # (B) Set the base station's position selected from the live survey window.
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
                               f"ECEF: ({x}cm, {y}cm, {z}cm)")

    def read_gps_loop(self):
        """
        Continuously read from the ZED-F9P. While in Survey-In mode, display survey info
        and check if the accuracy criteria are met. Once fixed mode is enabled, forward RTCM
        messages over UDP.
        """
        while rclpy.ok():
            try:
                msg = self.ubr.read()
                if msg[0] is None or msg[1] is None:
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
    if not node.started:
        node.close_resources()
        node.destroy_node()
        rclpy.shutdown()
        return
    rclpy.spin(node)
    node.close_resources()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
