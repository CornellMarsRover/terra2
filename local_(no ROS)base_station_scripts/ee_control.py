from pydualsense import pydualsense, TriggerModes
import argparse
import hidapi
import os
import time
import socket
import struct

#run 
#python ee_control.py --controller-path /dev/hidraw5


# Network settings
# UDP_IP   = "192.168.1.102"
# UDP_IP = "10.49.43.10" # JETSON IP
# UDP_IP = "10.49.15.204"

#UDP_IP = "192.168.1.69" # JETSON IP ON RADIO
UDP_IP = "192.168.1.69" # JETSON IP ON REDROVER WIFI
# UDP_IP = "128.253.53.211"
UDP_PORT = 5020 # 5010 - Drives, 5020 - Controller Arm, 5030 - Mini Arm
DEFAULT_CONTROLLER_PATH = os.environ.get("ARM_CONTROLLER_PATH", "/dev/hidraw5")

DPAD_NEUTRAL = 8
DPAD_UP = 0
DPAD_RIGHT = 2
DPAD_DOWN = 4
DPAD_LEFT = 6

def parse_args():
    parser = argparse.ArgumentParser(description="Send arm controller data over UDP.")
    parser.add_argument(
        "--controller-path",
        default=DEFAULT_CONTROLLER_PATH,
        help="DualSense HID path to use for the arm, e.g. /dev/hidraw5.",
    )
    return parser.parse_args()

def open_controller(controller_path: str):
    ds = pydualsense()

    if controller_path:
        def find_device_by_path():
            return hidapi.Device(path=os.fsencode(controller_path))

        ds._pydualsense__find_device = find_device_by_path

    ds.init()
    print(f"Using arm controller: {controller_path}")
    return ds

def read_dpad(ds_state):
    for attr in ("dpad", "Dpad", "DPAD", "hat", "hat_switch"):
        if hasattr(ds_state, attr):
            value = getattr(ds_state, attr)
            if isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= 8:
                return value

    up = bool(
        getattr(ds_state, "dpad_up", False)
        or getattr(ds_state, "DpadUp", False)
        or getattr(ds_state, "up", False)
    )
    right = bool(
        getattr(ds_state, "dpad_right", False)
        or getattr(ds_state, "DpadRight", False)
        or getattr(ds_state, "right", False)
    )
    down = bool(
        getattr(ds_state, "dpad_down", False)
        or getattr(ds_state, "DpadDown", False)
        or getattr(ds_state, "down", False)
    )
    left = bool(
        getattr(ds_state, "dpad_left", False)
        or getattr(ds_state, "DpadLeft", False)
        or getattr(ds_state, "left", False)
    )

    if up and right:
        return 1
    if right and down:
        return 3
    if down and left:
        return 5
    if left and up:
        return 7
    if up:
        return DPAD_UP
    if right:
        return DPAD_RIGHT
    if down:
        return DPAD_DOWN
    if left:
        return DPAD_LEFT
    return DPAD_NEUTRAL

def format_data_for_udp(ds_state):
    # Assuming ds_state is an instance of DSState or similar with attributes for each button/trigger
    # Convert button states to binary representation
    # Note: Adjust these according to the actual attributes/methods provided by pydualsense for button states
    lx = int(ds_state.LX) + 128
    ly = int(ds_state.LY) + 128
    rx = int(ds_state.RX) + 128
    ry = int(ds_state.RY) + 128
    square_button   = int(ds_state.square)
    cross_button    = int(ds_state.cross)
    circle_button   = int(ds_state.circle)
    triangle_button = int(ds_state.triangle)
    l1_button       = int(ds_state.L1)
    r1_button       = int(ds_state.R1)
    l2_button       = int(ds_state.L2)  
    r2_button       = int(ds_state.R2)
    
    hat_switch = read_dpad(ds_state)

    if l2_button < 30: 
        l2_button = 0
    if r2_button < 30: 
        r2_button = 0

    # Combine into a single list
    data_list = [lx, ly, rx, ry, l1_button, r1_button, l2_button, r2_button, square_button, cross_button, circle_button, triangle_button, hat_switch]
    return data_list

def main():
    # Initialize controller and network settings
    args = parse_args()
    ds = open_controller(args.controller_path)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        while True:
            # Update controller state
            #ds.update()
            #data = controller.read(64)
            # Format the controller state data for UDP transmission
            data_list = format_data_for_udp(ds.state)
            # ds.triggerR.setMode(TriggerModes.Rigid)
            # ds.triggerR.setForce(1, 255)
            for x in range(len(data_list)):
                if x == 12:
                    continue
                if data_list[x] != 0 and data_list[x] != 1:
                    data_list[x] -= 1
            print(data_list)


            #labelled_data = data
            # Convert data_list to bytes
            byte_data = bytearray(data_list)
            
            # Send data over UDP
            client_socket.sendto(byte_data, (UDP_IP, UDP_PORT))
            client_socket2.sendto(byte_data, (UDP_IP, 5040))
            
            # Set touchpad color
            ds.light.setColorI(15, 0, 90)
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping controller read...")
    finally:
        ds.close()  # Close the controller
        client_socket.close()

if __name__ == "__main__":
    main()
