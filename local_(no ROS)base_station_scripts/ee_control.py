import argparse
import os
import time
import socket
import struct

from controller_hid import open_dualsense, print_dualsense_devices

#run 
#python ee_control.py --controller-path /dev/hidraw5
#python ee_control.py --list-controllers
#python ee_control.py --controller-serial SERIAL_FROM_LIST


# Network settings
# UDP_IP   = "192.168.1.102"
# UDP_IP = "10.49.43.10" # JETSON IP
# UDP_IP = "10.49.15.204"

#UDP_IP = "192.168.1.69" # JETSON IP ON RADIO
UDP_IP = "192.168.1.69" # JETSON IP ON REDROVER WIFI
# UDP_IP = "128.253.53.211"
UDP_PORT = 5020 # 5010 - Drives, 5020 - Controller Arm, 5030 - Mini Arm
DEFAULT_CONTROLLER_PATH = os.environ.get("ARM_CONTROLLER_PATH")
DEFAULT_CONTROLLER_SERIAL = os.environ.get("ARM_CONTROLLER_SERIAL")
DEFAULT_CONTROLLER_INDEX = int(os.environ.get("ARM_CONTROLLER_INDEX", "1"))
LEGACY_CONTROLLER_PATH = "/dev/hidraw5"

DPAD_NEUTRAL = 8
DPAD_UP = 0
DPAD_RIGHT = 2
DPAD_DOWN = 4
DPAD_LEFT = 6


def load_pydualsense():
    try:
        from pydualsense import pydualsense
    except ModuleNotFoundError as exc:
        if exc.name != "pydualsense":
            raise
        raise SystemExit(
            "Missing Python dependency: pydualsense\n"
            "Install local controller dependencies with:\n"
            "  python3 -m pip install -r requirements.txt"
        ) from exc
    except OSError as exc:
        raise SystemExit(
            "Missing system HID library for pydualsense.\n"
            "Install it with:\n"
            "  sudo apt install libhidapi-hidraw0 libhidapi-libusb0"
        ) from exc
    return pydualsense

def parse_args():
    parser = argparse.ArgumentParser(description="Send arm controller data over UDP.")
    parser.add_argument(
        "--controller-path",
        default=DEFAULT_CONTROLLER_PATH,
        help="DualSense HID path to use for the arm, e.g. /dev/hidraw5.",
    )
    parser.add_argument(
        "--controller-serial",
        default=DEFAULT_CONTROLLER_SERIAL,
        help="DualSense serial number to use for the arm. Use --list-controllers to find it.",
    )
    parser.add_argument(
        "--controller-index",
        type=int,
        default=DEFAULT_CONTROLLER_INDEX,
        help="Zero-based DualSense index to use if no path/serial is provided.",
    )
    parser.add_argument(
        "--list-controllers",
        action="store_true",
        help="List detected DualSense HID devices and exit.",
    )
    return parser.parse_args()

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
    if args.list_controllers:
        print_dualsense_devices()
        return

    pydualsense = load_pydualsense()
    ds = open_dualsense(
        pydualsense,
        "arm",
        explicit_path=args.controller_path,
        serial=args.controller_serial,
        index=args.controller_index,
        legacy_path=LEGACY_CONTROLLER_PATH,
    )
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
