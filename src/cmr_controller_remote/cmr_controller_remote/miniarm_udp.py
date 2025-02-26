import rclpy
from rclpy.node import Node
import serial
import socket
import struct

SERIAL_PORT = "/dev/ttyACM0"  # Change this to match your system
BAUD_RATE = 115200
UDP_IP = "10.49.15.204"  # Localhost or target UDP address
UDP_PORT = 5030

class MiniArmUDPNode(Node):
    def __init__(self):
        super().__init__("mini_arm_udp_node")
        self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info("Mini Arm Serial-to-UDP Node Started")
        self.timer = self.create_timer(0.1, self.read_and_send)
        self.offsets = [0.0, 0.0, 0.0, 12.8, 12.6, 0.0]

    def read_and_send(self):
        try:
            line = self.serial_conn.readline().decode("utf-8").strip()
            if not line:
                return

            angles = [float(x) for x in line.split("|")]
            if len(angles) == 6:
                for i in range(len(angles)):
                    angles[i] += self.offsets[i]
                data = struct.pack("ffffff", *angles)
                self.udp_sock.sendto(data, (UDP_IP, UDP_PORT))
                self.get_logger().info(f"Sent angles: {angles}")
            else:
                self.get_logger().warn(f"Invalid data format: {line}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MiniArmUDPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
