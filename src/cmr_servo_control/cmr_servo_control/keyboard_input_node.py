import os
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

STEP = 5
EE8_MIN = 40
EE8_MAX = 150

KEY_INSTRUCTIONS = (
    "Keyboard servo control:\n"
    "  1/2 : ee13 -> 180° / 0°\n"
    "  3/4 : ee14 -> 180° / 0°\n"
    "  5/6 : ee8  +5° / -5°\n"
    "  0   : all servos to 90°\n"
    "  Ctrl+C : exit\n"
)


class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__('servo_keyboard_input_node')
        self.publisher = self.create_publisher(JointState, 'end_effector_joint_command', 10)
        self.ee_state = {'ee13': 0.0, 'ee14': 0.0, 'ee8': 90.0}
        self._running = True
        self._stdin_fd = sys.stdin.fileno()
        self._old_term_settings = termios.tcgetattr(self._stdin_fd)

        if not sys.stdin.isatty():
            self.get_logger().warn('Keyboard input node requires a terminal (TTY) attached to stdin.')
        else:
            tty.setraw(self._stdin_fd)
            self._thread = threading.Thread(target=self._key_loop, daemon=True)
            self._thread.start()
            self.get_logger().info('Keyboard input node started. Press keys in this terminal.')
            self.get_logger().info(KEY_INSTRUCTIONS)

    def _publish_state(self):
        msg = JointState()
        msg.name = ['ee13', 'ee14', 'ee8']
        msg.position = [self.ee_state['ee13'], self.ee_state['ee14'], self.ee_state['ee8']]
        self.publisher.publish(msg)

    def _handle_key(self, key: str):
        if key == '1':
            self.ee_state['ee13'] = 180.0
        elif key == '2':
            self.ee_state['ee13'] = 0.0
        elif key == '3':
            self.ee_state['ee14'] = 180.0
        elif key == '4':
            self.ee_state['ee14'] = 0.0
        elif key == '6':
            self.ee_state['ee8'] = min(EE8_MAX, self.ee_state['ee8'] + STEP)
        elif key == '5':
            self.ee_state['ee8'] = max(EE8_MIN, self.ee_state['ee8'] - STEP)
        elif key == '0':
            self.ee_state['ee13'] = 90.0
            self.ee_state['ee14'] = 90.0
            self.ee_state['ee8'] = 90.0
        else:
            return

        self.get_logger().info(f'Keyboard command: {key} -> {self.ee_state}')
        self._publish_state()

    def _key_loop(self):
        try:
            while self._running and rclpy.ok():
                reads, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not reads:
                    continue
                key = os.read(self._stdin_fd, 1).decode('utf-8', errors='ignore')
                if key == '\x03':
                    self.get_logger().info('Ctrl+C received, shutting down keyboard node.')
                    rclpy.shutdown()
                    break
                self._handle_key(key)
        finally:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term_settings)

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_thread') and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if sys.stdin.isatty():
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
