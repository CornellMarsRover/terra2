import asyncio
import threading
from typing import Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .CMR_CANFD import FdCanInterface, ServoController

SERVO_NAMES = ['ee13', 'ee14', 'ee8']
SERVO_IDS = {'ee13': 13, 'ee14': 14, 'ee8': 8}


class ServoHwNode(Node):
    def __init__(self):
        super().__init__('servo_hw_node')

        self.declare_parameter('can_port', '/dev/ttyTHS0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('servo_can_id', 24)

        self.can_port = self.get_parameter('can_port').value
        self.baud = self.get_parameter('baud').value
        self.servo_can_id = self.get_parameter('servo_can_id').value

        self.get_logger().info(f'Opening CAN interface on {self.can_port} at {self.baud} baud')

        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.loop_thread.start()

        self.fd = FdCanInterface(port=self.can_port, baud=self.baud)
        self.servos: Dict[str, ServoController] = {}

        startup_future = asyncio.run_coroutine_threadsafe(self._startup(), self.loop)
        startup_future.result(timeout=10.0)

        self.subscription = self.create_subscription(
            JointState,
            'end_effector_joint_command',
            self.command_callback,
            10,
        )
        self.get_logger().info('Servo hardware node ready and subscribed to end_effector_joint_command')

    async def _startup(self):
        await self.fd.open()
        await self.fd.configure_bus()
        for name in SERVO_NAMES:
            self.servos[name] = ServoController(
                can=self.fd,
                servo_id=SERVO_IDS[name],
                can_id=self.servo_can_id,
            )

    def command_callback(self, msg: JointState):
        positions = {name: pos for name, pos in zip(msg.name, msg.position)}
        if not positions:
            return
        for name, servo in self.servos.items():
            if name in positions:
                try:
                    angle = int(positions[name])
                except (TypeError, ValueError):
                    self.get_logger().warn(f'Invalid position for {name}: {positions[name]}')
                    continue
                self.get_logger().info(f'Publishing {angle}° to {name}')
                asyncio.run_coroutine_threadsafe(servo.go_to_position(angle), self.loop)

    def destroy_node(self):
        if hasattr(self, 'fd') and self.fd is not None:
            self.get_logger().info('Shutting down CAN interface')
            close_future = asyncio.run_coroutine_threadsafe(self.fd.close(), self.loop)
            try:
                close_future.result(timeout=5.0)
            except Exception as exc:
                self.get_logger().warn(f'Exception while closing CAN interface: {exc}')
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoHwNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
