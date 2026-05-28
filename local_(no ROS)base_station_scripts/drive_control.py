from pydualsense import pydualsense
import argparse
import hidapi
import os
import time
import socket

#run
#python drive_control.py --controller-path /dev/hidraw4

UDP_IP = "192.168.1.69"   # Jetson / rover IP
UDP_PORT = 5005        # 5010 = drives
DEFAULT_CONTROLLER_PATH = os.environ.get("DRIVE_CONTROLLER_PATH", "/dev/hidraw4")


def parse_args():
    parser = argparse.ArgumentParser(description="Send drive controller data over UDP.")
    parser.add_argument(
        "--controller-path",
        default=DEFAULT_CONTROLLER_PATH,
        help="DualSense HID path to use for drives, e.g. /dev/hidraw4.",
    )
    return parser.parse_args()


def open_controller(controller_path: str):
    ds = pydualsense()

    if controller_path:
        def find_device_by_path():
            return hidapi.Device(path=os.fsencode(controller_path))

        ds._pydualsense__find_device = find_device_by_path

    ds.init()
    print(f"Using drive controller: {controller_path}")
    return ds


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
    ds = open_controller(args.controller_path)

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
