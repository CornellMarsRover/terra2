from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import ControllerReading
from cmr_msgs.msg import MiniArmDegree
from sensor_msgs.msg import Joy
import socket
import rclpy
from rclpy.node import Node
import struct


UDP_IP = "0.0.0.0"  # Listen on all available IPs

UDP_PORT_DRIVES = 5010
UDP_PORT_DRIVES_LOCAL = 5005
UDP_PORT_ARM = 5020
UDP_PORT_MINI_ARM = 5030

DRIVE_PACKET_LENGTH = 13
MINI_ARM_PACKET_LENGTH = struct.calcsize("ffffff")

# Important:
# Joystick drive packets should NOT publish to /cmd_vel_drives.
# /cmd_vel_drives is reserved for autonomy or another intentional source.
# Manual joystick driving publishes:
#   /drives_controller/cmd_vel
#   /drives_controller/cmd_buttons

directions = {
    "neutral": 8,
    "N": 0,
    "NE": 1,
    "E": 2,
    "SE": 3,
    "S": 4,
    "SW": 5,
    "W": 6,
    "NW": 7,
}


def create_udp_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setblocking(False)
    sock.bind((UDP_IP, port))
    return sock


print("Connecting Drive UDP sockets")
drive_sockets = {
    UDP_PORT_DRIVES: create_udp_socket(UDP_PORT_DRIVES),
    UDP_PORT_DRIVES_LOCAL: create_udp_socket(UDP_PORT_DRIVES_LOCAL),
}

for port, sock in drive_sockets.items():
    print(f"Drive port {port}: {sock}")

print("Connecting Arm")
arm_sock = create_udp_socket(UDP_PORT_ARM)
print(arm_sock)

print("Connecting Mini Arm")
mini_arm_sock = create_udp_socket(UDP_PORT_MINI_ARM)
print(mini_arm_sock)


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__("cmd_vel_publisher")

        # Manual joystick drive topics.
        self.publisher_ = self.create_publisher(
            TwistStamped,
            "/drives_controller/cmd_vel",
            10,
        )

        self.button_publisher_ = self.create_publisher(
            ControllerReading,
            "/drives_controller/cmd_buttons",
            10,
        )

        # Arm topics.
        self.arm_publisher_ = self.create_publisher(
            Joy,
            "/arm_controller/cmd_vel",
            10,
        )

        self.arm_button_publisher_ = self.create_publisher(
            ControllerReading,
            "/arm_controller/cmd_buttons",
            10,
        )

        self.mini_arm_publisher = self.create_publisher(
            MiniArmDegree,
            "/mini_arm_controller/cmd_pos",
            10,
        )

        self.timer = self.create_timer(0.02, self.publish_msg)
        self._last_drive_button_state = None

        self.logger = self.get_logger()
        drive_ports = ", ".join(str(port) for port in sorted(drive_sockets))

        self.logger.info(
            f"Listening for drive UDP packets on 0.0.0.0 ports [{drive_ports}], "
            f"arm on {UDP_PORT_ARM}, mini-arm on {UDP_PORT_MINI_ARM}"
        )

        self.logger.info(
            "Joystick UDP bridge will publish manual drive topics only: "
            "/drives_controller/cmd_vel and /drives_controller/cmd_buttons. "
            "/cmd_vel_drives is not published by connect.py."
        )

    def publish_msg(self):
        for port, sock in drive_sockets.items():
            packet = self.recv_latest(sock)
            if packet is None:
                continue

            data, addr = packet

            if not self.validate_drive_packet(data, port, addr):
                continue

            lx, ly, rx, ry = self.parse_controller_data(data)
            button_array, dpad = self.parse_button_data(data)

            msg = self.create_twist_stamped(
                [lx, ly, rx, ry],
                frame_id=f"udp:{port}",
            )

            button_msg = self.create_button_message(button_array, dpad)

            # Manual joystick path only.
            # Do NOT publish joystick data to /cmd_vel_drives.
            self.publisher_.publish(msg)
            self.button_publisher_.publish(button_msg)

            self.log_drive_button_changes(button_array)

            self.logger.info(
                f"Drive packet from {addr[0]}:{addr[1]} on {port} -> "
                f"lx={lx:.3f} ly={ly:.3f} rx={rx:.3f} ry={ry:.3f}",
                throttle_duration_sec=1.0,
            )

        arm_packet = self.recv_latest(arm_sock)
        if arm_packet is not None:
            arm_data, addr = arm_packet

            if self.validate_drive_packet(arm_data, UDP_PORT_ARM, addr, kind="arm"):
                lx, ly, rx, ry = self.parse_controller_data(arm_data)
                arm_button_array, dpad = self.parse_button_data(arm_data)

                arm_msg = self.create_arm_joy_message(
                    [
                        lx,
                        ly,
                        arm_button_array[2],
                        rx,
                        ry,
                        arm_button_array[3],
                        arm_button_array[5],
                        arm_button_array[6],
                        arm_button_array[7],
                        arm_button_array[4],
                        arm_button_array[0],
                        arm_button_array[1],
                    ],
                    frame_id=f"udp:{UDP_PORT_ARM}",
                )

                arm_button_msg = self.create_arm_button_message(arm_button_array)

                self.arm_publisher_.publish(arm_msg)
                self.arm_button_publisher_.publish(arm_button_msg)

        mini_arm_packet = self.recv_latest(mini_arm_sock)
        if mini_arm_packet is not None:
            mini_arm_data_raw, addr = mini_arm_packet

            if len(mini_arm_data_raw) != MINI_ARM_PACKET_LENGTH:
                self.logger.warn(
                    f"Ignoring mini-arm packet from {addr[0]}:{addr[1]} on "
                    f"{UDP_PORT_MINI_ARM}: expected {MINI_ARM_PACKET_LENGTH} bytes, "
                    f"got {len(mini_arm_data_raw)}",
                    throttle_duration_sec=2.0,
                )
                return

            mini_arm_data = struct.unpack("ffffff", mini_arm_data_raw)
            mini_arm_msg = self.create_mini_arm_message(mini_arm_data)
            self.mini_arm_publisher.publish(mini_arm_msg)

    def recv_latest(self, sock):
        latest_packet = None

        try:
            while True:
                latest_packet = sock.recvfrom(1024)
        except BlockingIOError:
            return latest_packet

    def validate_drive_packet(self, data, port, addr, kind="drive"):
        if len(data) != DRIVE_PACKET_LENGTH:
            self.logger.warn(
                f"Ignoring {kind} packet from {addr[0]}:{addr[1]} on {port}: "
                f"expected exactly {DRIVE_PACKET_LENGTH} bytes, got {len(data)}",
                throttle_duration_sec=2.0,
            )
            return False

        return True

    def create_mini_arm_message(self, mini_arm_data):
        msg = MiniArmDegree()
        msg.base_angle = mini_arm_data[0]
        msg.shoulder_angle = mini_arm_data[1]
        msg.elbow_angle = mini_arm_data[2]
        msg.first_rotate_angle = mini_arm_data[3]
        msg.tilt_angle = mini_arm_data[4]
        msg.second_rotate_angle = mini_arm_data[5]
        return msg

    def axis_map_for_arm(self, axis):
        if axis >= 1:
            result = -1
        elif axis <= -1:
            result = 1
        else:
            result = 0

        return result

    def create_twist_stamped(self, velocities, frame_id=""):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id

        twist_msg.twist.linear.x = velocities[0]
        twist_msg.twist.linear.y = velocities[1]
        twist_msg.twist.angular.x = velocities[2]
        twist_msg.twist.angular.y = velocities[3]

        return twist_msg

    def create_arm_joy_message(self, velocities, frame_id=""):
        joy_msg = Joy()

        lx, ly, l2, rx, ry, r2, x, circle, triangle, square, l1, r1 = velocities

        lx_val = self.axis_map_for_arm(lx)
        ly_val = self.axis_map_for_arm(ly)
        rx_val = self.axis_map_for_arm(rx)
        ry_val = self.axis_map_for_arm(ry)

        if l2:
            l2 = -1
        else:
            l2 = 1

        if r2:
            r2 = -1
        else:
            r2 = 1

        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = frame_id or "joy"

        joy_msg.axes = [
            float(lx_val),
            float(ly_val),
            float(l2),
            float(rx_val),
            float(ry_val),
            float(r2),
            0.0,
            0.0,
        ]

        joy_msg.buttons = [
            x,
            circle,
            triangle,
            square,
            l1,
            r1,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]

        return joy_msg

    def create_button_message(self, button_array, dpad):
        button_msg = ControllerReading()
        button_msg.button_array = button_array
        button_msg.dpad = dpad
        return button_msg

    def create_arm_button_message(self, button_array):
        button_msg = ControllerReading()
        button_msg.button_array = button_array
        return button_msg

    def decode_drive_buttons(self, button_array):
        buttons = list(button_array)

        names = [
            "L1",
            "R1",
            "L2",
            "R2",
            "SQUARE",
            "X",
            "CIRCLE",
            "TRIANGLE",
        ]

        state = {name: 0 for name in names}

        for index, name in enumerate(names):
            if index < len(buttons):
                state[name] = int(buttons[index])

        return state

    def log_drive_button_changes(self, button_array):
        state = self.decode_drive_buttons(button_array)

        if self._last_drive_button_state is None:
            self._last_drive_button_state = state

            pressed = [name for name, value in state.items() if value]

            self.logger.info(
                "Drive buttons initialized: "
                f"pressed={pressed if pressed else ['none']} "
                f"L2={state['L2']} R2={state['R2']}",
                throttle_duration_sec=1.0,
            )
            return

        changed = []

        for name, value in state.items():
            previous = self._last_drive_button_state.get(name, 0)

            if value != previous:
                if name in {"L2", "R2"}:
                    changed.append(f"{name}={value}")
                else:
                    changed.append(f"{name}={'pressed' if value else 'released'}")

        if changed:
            self.logger.info("Drive button update: " + ", ".join(changed))

        self._last_drive_button_state = state

    def parse_controller_data(self, raw_data):
        lx_raw = raw_data[0]
        ly_raw = raw_data[1]
        rx_raw = raw_data[2]
        ry_raw = raw_data[3]

        lx = self.scale_value(lx_raw, 0, 255, -2.5, 2.5)
        ly = self.scale_value(ly_raw, 0, 255, -2.5, 2.5)
        rx = self.scale_value(rx_raw, 0, 255, -2.5, 2.5)
        ry = self.scale_value(ry_raw, 0, 255, -2.5, 2.5)

        return float(lx), float(ly), float(rx), float(ry)

    def parse_button_data(self, raw_data):
        buttons = list(raw_data[4:12])

        dpad_raw = raw_data[12]
        dpad = dpad_raw if 0 <= dpad_raw <= 8 else directions["neutral"]

        return buttons, dpad

    def scale_value(self, value, old_min, old_max, new_min, new_max):
        return ((value - old_min) / (old_max - old_min)) * (
            new_max - new_min
        ) + new_min


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = CmdVelPublisher()

    try:
        rclpy.spin(cmd_vel_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_publisher.destroy_node()

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()