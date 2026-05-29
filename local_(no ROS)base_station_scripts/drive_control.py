import argparse
import os
import time
import socket

from controller_hid import open_dualsense, print_dualsense_devices

#run
#python drive_control.py --controller-path /dev/hidraw4
#python drive_control.py --list-controllers
#python drive_control.py --controller-serial SERIAL_FROM_LIST

UDP_IP = "192.168.1.69"   # Jetson / rover IP
UDP_PORT = 5005        # 5010 = drives
DEFAULT_CONTROLLER_PATH = os.environ.get("DRIVE_CONTROLLER_PATH")
DEFAULT_CONTROLLER_SERIAL = os.environ.get("DRIVE_CONTROLLER_SERIAL")
DEFAULT_CONTROLLER_INDEX = int(os.environ.get("DRIVE_CONTROLLER_INDEX", "0"))
LEGACY_CONTROLLER_PATH = "/dev/hidraw4"


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
    parser = argparse.ArgumentParser(description="Send drive controller data over UDP.")
    parser.add_argument(
        "--controller-path",
        default=DEFAULT_CONTROLLER_PATH,
        help="DualSense HID path to use for drives, e.g. /dev/hidraw4.",
    )
    parser.add_argument(
        "--controller-serial",
        default=DEFAULT_CONTROLLER_SERIAL,
        help="DualSense serial number to use for drives. Use --list-controllers to find it.",
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


def clamp_byte(value: int) -> int:
    return max(0, min(255, int(value)))


def stick_to_udp_byte(value: int) -> int:
    # pydualsense stick values are typically centered around 0.
    # Shift into the 0..255 format expected by connect_node.
    return clamp_byte(value + 128)


def trigger_to_udp_byte(value: int, deadband: int = 30) -> int:
    value = clamp_byte(value)
    if value < deadband:
        return 0
    return value


def format_data_for_udp(ds_state) -> bytearray:
    lx = stick_to_udp_byte(ds_state.LX)
    ly = stick_to_udp_byte(ds_state.LY)
    rx = stick_to_udp_byte(ds_state.RX)
    ry = stick_to_udp_byte(ds_state.RY)

    square_button = int(bool(ds_state.square))
    cross_button = int(bool(ds_state.cross))
    circle_button = int(bool(ds_state.circle))
    triangle_button = int(bool(ds_state.triangle))

    l1_button = int(bool(ds_state.L1))
    r1_button = int(bool(ds_state.R1))

    l2_value = trigger_to_udp_byte(ds_state.L2)
    r2_value = trigger_to_udp_byte(ds_state.R2)

    hat_switch = 0

    data_list = [
        lx,
        ly,
        rx,
        ry,
        l1_button,
        r1_button,
        l2_value,
        r2_value,
        square_button,
        cross_button,
        circle_button,
        triangle_button,
        hat_switch,
    ]
    return bytearray(data_list)


def main():
    args = parse_args()
    if args.list_controllers:
        print_dualsense_devices()
        return

    pydualsense = load_pydualsense()
    ds = open_dualsense(
        pydualsense,
        "drive",
        explicit_path=args.controller_path,
        serial=args.controller_serial,
        index=args.controller_index,
        legacy_path=LEGACY_CONTROLLER_PATH,
    )

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        while True:
            byte_data = format_data_for_udp(ds.state)

            print(list(byte_data))

            client_socket.sendto(byte_data, (UDP_IP, UDP_PORT))
            client_socket2.sendto(byte_data, (UDP_IP, 5040))

            ds.light.setColorI(15, 0, 90)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping controller read...")

    finally:
        try:
            stop_packet = bytearray([128, 128, 128, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            client_socket.sendto(stop_packet, (UDP_IP, UDP_PORT))
            client_socket2.sendto(stop_packet, (UDP_IP, 5040))
        except Exception:
            pass

        ds.close()
        client_socket.close()
        client_socket2.close()


if __name__ == "__main__":
    main()
