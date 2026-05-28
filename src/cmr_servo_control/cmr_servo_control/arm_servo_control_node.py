import asyncio
import threading
from typing import Dict

import rclpy
from rclpy.node import Node

from cmr_msgs.msg import ControllerReading

from .CMR_CANFD import FdCanInterface, ServoController
from .end_effector_servo_mapper import (
    EndEffectorServoMapper,
    SERVO_IDS,
)


class ArmServoControlNode(Node):
    def __init__(self):
        super().__init__("arm_servo_control_node")

        self.declare_parameter("can_port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("servo_can_id", 25)
        self.declare_parameter("command_topic", "/arm_controller/cmd_buttons")

        self.can_port = self.get_parameter("can_port").value
        self.baud = self.get_parameter("baud").value
        self.servo_can_id = self.get_parameter("servo_can_id").value
        self.command_topic = self.get_parameter("command_topic").value

        self.mapper = EndEffectorServoMapper()
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.loop_thread.start()

        self.fd = FdCanInterface(port=self.can_port, baud=self.baud)
        self.servos: Dict[str, ServoController] = {}

        self.get_logger().info(
            f"Opening servo CAN interface on {self.can_port} at {self.baud} baud"
        )
        startup_future = asyncio.run_coroutine_threadsafe(self._startup(), self.loop)
        startup_future.result(timeout=10.0)

        self.subscription = self.create_subscription(
            ControllerReading,
            self.command_topic,
            self.command_callback,
            10,
        )
        self.get_logger().info(
            f"Arm servo control ready: topic={self.command_topic}, "
            f"servo_can_id={self.servo_can_id}"
        )

    async def _startup(self):
        await self.fd.open()
        await self.fd.configure_bus()
        for name, servo_id in SERVO_IDS.items():
            self.servos[name] = ServoController(
                can=self.fd,
                servo_id=servo_id,
                can_id=self.servo_can_id,
            )

    def command_callback(self, msg: ControllerReading):
        commands = self.mapper.update(msg.button_array, msg.dpad)
        for name, angle in commands:
            servo = self.servos.get(name)
            if servo is None:
                self.get_logger().warn(f"No configured servo for {name}")
                continue
            self.get_logger().info(f"Commanding {name} -> {angle} deg")
            future = asyncio.run_coroutine_threadsafe(
                servo.go_to_position(angle),
                self.loop,
            )
            future.add_done_callback(
                lambda done, servo_name=name: self._log_command_result(
                    servo_name,
                    done,
                )
            )

    def _log_command_result(self, servo_name, future):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f"Servo command failed for {servo_name}: {exc}")

    def destroy_node(self):
        if hasattr(self, "fd") and self.fd is not None:
            self.get_logger().info("Shutting down servo CAN interface")
            close_future = asyncio.run_coroutine_threadsafe(self.fd.close(), self.loop)
            try:
                close_future.result(timeout=5.0)
            except Exception as exc:
                self.get_logger().warn(f"Exception while closing CAN interface: {exc}")
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmServoControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
