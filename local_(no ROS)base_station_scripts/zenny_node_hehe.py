from pydualsense import pydualsense
import time
import socket


UDP_IP = "192.168.1.69"  # Jetson / rover IP
UDP_PORT = 5005          # connect.py now accepts 5005 and 5010
SEND_PERIOD_S = 0.02     # 50 Hz for smoother teleop


def clamp_byte(value: int) -> int:
    return max(0, min(255, int(value)))


def stick_to_udp_byte(value: int) -> int:
    # pydualsense stick values are centered around 0.
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

    hat_switch = 8  # neutral

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


def make_stop_packet() -> bytearray:
    return bytearray([128, 128, 128, 128, 0, 0, 0, 0, 0, 0, 0, 0, 8])


def main():
    ds = pydualsense()
    ds.init()

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        while True:
            byte_data = format_data_for_udp(ds.state)
            print(list(byte_data))
            client_socket.sendto(byte_data, (UDP_IP, UDP_PORT))
            ds.light.setColorI(15, 0, 90)
            time.sleep(SEND_PERIOD_S)

    except KeyboardInterrupt:
        print("Stopping controller read...")

    finally:
        try:
            client_socket.sendto(make_stop_packet(), (UDP_IP, UDP_PORT))
        except Exception:
            pass

        ds.close()
        client_socket.close()


if __name__ == "__main__":
    main()
