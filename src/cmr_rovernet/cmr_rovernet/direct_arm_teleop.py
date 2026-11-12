"""Direct joystick velocity control for arm moteus controllers 9 through 14.

This node intentionally bypasses MoveIt Servo and Cartesian inverse kinematics.
Each stick axis controls exactly one moteus controller, matching the supplied
standalone pygame program.
"""

import math
import time

import rclpy
import toml
from rclpy.node import Node
from sensor_msgs.msg import Joy

from cmr_rovernet.rovernet_utils import (
    cycle_moteus_sync,
    init_moteus_loop,
    make_moteus_transport_and_controllers_sync,
)


JOINT_MOTOR_IDS = {
    "base_joint": 9,
    "shoulder_joint": 10,
    "elbow_joint": 11,
    "wrist_rotate": 12,
    "wrist_twist": 13,
    "wrist_rotate_two": 14,
}

# (DualSense axis index, joint name, invert axis)
JOYSTICK_AXES = (
    (0, "base_joint", False),
    (1, "shoulder_joint", True),
    (3, "elbow_joint", False),
    (2, "wrist_rotate", True),
)

def apply_deadzone(value: float, deadzone: float) -> float:
    """Remove the center deadzone and rescale the remaining stick travel."""
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return math.copysign(min(scaled, 1.0), value)


class DirectArmTeleop(Node):
    def __init__(self):
        super().__init__("direct_arm_teleop")

        self.declare_parameter("config_path", "")
        self.declare_parameter("joy_topic", "/arm_controller/cmd_vel")
        self.declare_parameter("max_velocity_rev_s", 5.0)
        self.declare_parameter("accel_limit_rev_s2", 10.0)
        self.declare_parameter("maximum_torque", 0.7)
        self.declare_parameter("joystick_deadzone", 0.12)
        self.declare_parameter("joy_timeout_sec", 0.25)

        config_path = str(self.get_parameter("config_path").value)
        config = toml.load(config_path).get("node", {}) if config_path else {}
        self.can_port = str(config.get("can_port", "/dev/ttyACM0"))
        self.max_velocity = float(self.get_parameter("max_velocity_rev_s").value)
        self.accel_limit = float(self.get_parameter("accel_limit_rev_s2").value)
        self.maximum_torque = float(self.get_parameter("maximum_torque").value)
        self.deadzone = float(self.get_parameter("joystick_deadzone").value)
        self.joy_timeout = float(self.get_parameter("joy_timeout_sec").value)

        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("joystick_deadzone must be in [0, 1)")
        if self.max_velocity <= 0.0 or self.accel_limit <= 0.0:
            raise ValueError("velocity and acceleration limits must be positive")

        self.transport, self.controllers = make_moteus_transport_and_controllers_sync(
            self.can_port, JOINT_MOTOR_IDS
        )
        self.velocities = {joint_name: 0.0 for joint_name in JOINT_MOTOR_IDS}
        self.last_joy_time = None
        self.stopped = False

        joy_topic = str(self.get_parameter("joy_topic").value)
        self.joy_subscription = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.control_timer = self.create_timer(0.02, self.control_cycle)

        self.get_logger().info(f"Direct joint teleop using {self.can_port}")
        self.get_logger().info(
            "Controls: left X/Y -> IDs 9/10; right X/Y -> IDs 11/12; "
            "IDs 13/14 held at zero velocity"
        )
        self.get_logger().info(
            f"Speed limit {self.max_velocity:.2f} rev/s, acceleration limit "
            f"{self.accel_limit:.2f} rev/s^2, deadzone {self.deadzone:.2f}"
        )

    def joy_callback(self, message: Joy):
        if len(message.axes) < 4:
            self.get_logger().error(
                f"Joystick has only {len(message.axes)} axes; at least 4 are required.",
                throttle_duration_sec=1.0,
            )
            self.velocities = {joint_name: 0.0 for joint_name in JOINT_MOTOR_IDS}
            return

        for axis_index, joint_name, invert in JOYSTICK_AXES:
            value = float(message.axes[axis_index])
            if invert:
                value = -value
            self.velocities[joint_name] = (
                apply_deadzone(value, self.deadzone) * self.max_velocity
            )

        # The supplied controller program reserves these for sliders. Keep them
        # stationary until their desired controller mapping is specified.
        self.velocities["wrist_twist"] = 0.0
        self.velocities["wrist_rotate_two"] = 0.0
        self.last_joy_time = time.monotonic()

    def control_cycle(self):
        if self.last_joy_time is None or time.monotonic() - self.last_joy_time > self.joy_timeout:
            commanded_velocities = {joint_name: 0.0 for joint_name in JOINT_MOTOR_IDS}
        else:
            commanded_velocities = self.velocities

        commands = [
            self.controllers[joint_name].make_position(
                position=math.nan,
                velocity=commanded_velocities[joint_name],
                velocity_limit=self.max_velocity,
                accel_limit=self.accel_limit,
                maximum_torque=self.maximum_torque,
                watchdog_timeout=self.joy_timeout,
                query=False,
            )
            for joint_name in JOINT_MOTOR_IDS
        ]

        try:
            cycle_moteus_sync(self.transport, commands, timeout_s=1.0)
        except Exception as exc:
            self.get_logger().error(
                f"CAN command cycle failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=1.0,
            )
            return

    def stop_motors(self):
        if self.stopped:
            return
        self.stopped = True
        try:
            commands = [controller.make_stop() for controller in self.controllers.values()]
            cycle_moteus_sync(self.transport, commands, timeout_s=1.0)
            self.get_logger().info("STOP sent to arm Moteus IDs 9-14")
        except Exception as exc:
            self.get_logger().error(f"Failed to send final STOP: {type(exc).__name__}: {exc}")


def main(args=None):
    rclpy.init(args=args)
    init_moteus_loop()
    node = DirectArmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
