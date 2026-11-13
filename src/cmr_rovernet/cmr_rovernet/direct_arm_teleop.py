"""Direct joystick velocity control for arm moteus controllers 9 through 14.

This node intentionally bypasses MoveIt Servo and Cartesian inverse kinematics.
Each stick axis controls exactly one moteus controller, matching the supplied
standalone pygame program.
"""

import json
import math
import time
from pathlib import Path

import moteus
import rclpy
import toml
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

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

REPLAY_SEQUENCE = (
    "C_above",
    "C_pressed",
    "C_above",
    "M_above",
    "M_pressed",
    "M_above",
    "R_above",
    "R_pressed",
    "R_above",
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
        self.declare_parameter("taught_positions_path", "")
        self.declare_parameter("replay_settle_sec", 0.6)

        config_path = str(self.get_parameter("config_path").value)
        config = toml.load(config_path).get("node", {}) if config_path else {}
        self.can_port = str(config.get("can_port", "/dev/ttyACM0"))
        self.max_velocity = float(self.get_parameter("max_velocity_rev_s").value)
        self.accel_limit = float(self.get_parameter("accel_limit_rev_s2").value)
        self.maximum_torque = float(self.get_parameter("maximum_torque").value)
        self.deadzone = float(self.get_parameter("joystick_deadzone").value)
        self.joy_timeout = float(self.get_parameter("joy_timeout_sec").value)
        self.replay_settle_sec = float(self.get_parameter("replay_settle_sec").value)
        taught_positions_path = str(self.get_parameter("taught_positions_path").value)
        if not taught_positions_path:
            if not config_path:
                raise ValueError("config_path or taught_positions_path is required")
            taught_positions_path = str(
                Path(config_path).with_name("taught_arm_positions.json")
            )
        self.taught_positions = self.load_taught_positions(taught_positions_path)

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
        self.runtime_a = None
        self.replay_targets = []
        self.replay_names = []
        self.replay_index = -1
        self.replay_deadline = 0.0
        self.position_hold_target = None

        joy_topic = str(self.get_parameter("joy_topic").value)
        self.joy_subscription = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.direct_position_subscription = self.create_subscription(
            Float64MultiArray,
            "/arm/direct_motor_positions",
            self.direct_position_callback,
            10,
        )
        self.capture_service = self.create_service(
            Trigger, "/arm/capture_joint_positions", self.capture_joint_positions
        )
        self.set_runtime_a_service = self.create_service(
            Trigger, "/arm/set_runtime_a", self.set_runtime_a
        )
        self.type_cmr_service = self.create_service(
            Trigger, "/arm/type_cmr", self.start_cmr_replay
        )
        self.go_to_taught_a_service = self.create_service(
            Trigger, "/arm/go_to_taught_a", self.go_to_taught_a
        )
        self.cancel_replay_service = self.create_service(
            Trigger, "/arm/cancel_replay", self.cancel_replay
        )
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
        self.get_logger().info(
            "Capture IDs 9-11 with service /arm/capture_joint_positions"
        )
        self.get_logger().info(
            "Replay: call /arm/type_cmr; /arm/set_runtime_a is optional after encoder reset"
        )
        self.get_logger().info(
            "Direct position topic /arm/direct_motor_positions expects [ID9, ID10, ID11] revolutions"
        )

    @staticmethod
    def load_taught_positions(path):
        with open(path, "r", encoding="utf-8") as positions_file:
            data = json.load(positions_file)
        positions = data.get("positions", {})
        required = {"A", *REPLAY_SEQUENCE}
        missing = sorted(required - positions.keys())
        if missing:
            raise ValueError(f"Taught positions file is missing: {missing}")
        for name in required:
            if len(positions[name]) != 3 or not all(
                math.isfinite(float(value)) for value in positions[name]
            ):
                raise ValueError(f"Invalid taught position: {name}")
        return {
            name: {motor_id: float(value) for motor_id, value in zip((9, 10, 11), values)}
            for name, values in positions.items()
        }

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

        if self.position_hold_target is not None and not self.replay_targets:
            if any(abs(self.velocities[name]) > 1e-6 for name in JOINT_MOTOR_IDS):
                self.position_hold_target = None
                self.get_logger().info("Joystick input released replay position hold")

    def direct_position_callback(self, message: Float64MultiArray):
        if len(message.data) != 3:
            self.get_logger().error(
                "/arm/direct_motor_positions requires exactly 3 values: [ID9, ID10, ID11]"
            )
            return
        if not all(math.isfinite(float(value)) for value in message.data):
            self.get_logger().error("Ignoring non-finite direct motor position target")
            return
        if self.replay_targets:
            self.get_logger().error("Ignoring direct target while CMR replay is active")
            return
        if any(abs(self.velocities[name]) > 1e-6 for name in JOINT_MOTOR_IDS):
            self.get_logger().error("Center the controller before sending a direct target")
            return

        self.runtime_a = None
        self.position_hold_target = {
            motor_id: float(value)
            for motor_id, value in zip((9, 10, 11), message.data)
        }
        self.get_logger().warning(
            "Direct motor target accepted: "
            f"9={self.position_hold_target[9]:.9f}, "
            f"10={self.position_hold_target[10]:.9f}, "
            f"11={self.position_hold_target[11]:.9f}"
        )

    def control_cycle(self):
        self.advance_replay_if_ready()

        if self.replay_targets or self.position_hold_target is not None:
            target = (
                self.replay_targets[self.replay_index]
                if self.replay_targets
                else self.position_hold_target
            )
            commands = [
                self.controllers[joint_name].make_position(
                    position=target[motor_id] if motor_id in target else math.nan,
                    velocity=0.0,
                    velocity_limit=self.max_velocity,
                    accel_limit=self.accel_limit,
                    maximum_torque=self.maximum_torque,
                    watchdog_timeout=self.joy_timeout,
                    query=False,
                )
                for joint_name, motor_id in JOINT_MOTOR_IDS.items()
            ]
        else:
            commands = self.make_velocity_commands()

        try:
            cycle_moteus_sync(self.transport, commands, timeout_s=1.0)
        except Exception as exc:
            self.get_logger().error(
                f"CAN command cycle failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=1.0,
            )

    def make_velocity_commands(self):
        if self.last_joy_time is None or time.monotonic() - self.last_joy_time > self.joy_timeout:
            commanded_velocities = {joint_name: 0.0 for joint_name in JOINT_MOTOR_IDS}
        else:
            commanded_velocities = self.velocities

        return [
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

    def advance_replay_if_ready(self):
        if not self.replay_targets or time.monotonic() < self.replay_deadline:
            return

        previous_target = self.replay_targets[self.replay_index]
        self.replay_index += 1
        if self.replay_index >= len(self.replay_targets):
            self.position_hold_target = previous_target
            self.replay_targets = []
            self.replay_names = []
            self.replay_index = -1
            self.get_logger().info("CMR replay complete; holding at R_above")
            return

        self.set_replay_deadline(previous_target, self.replay_targets[self.replay_index])
        self.get_logger().info(f"Replay target: {self.replay_names[self.replay_index]}")

    def set_replay_deadline(self, start, target):
        max_travel = max(abs(target[motor_id] - start[motor_id]) for motor_id in (9, 10, 11))
        self.replay_deadline = (
            time.monotonic() + max_travel / self.max_velocity + self.replay_settle_sec
        )

    def query_positions(self):
        commands = [
            self.controllers[joint_name].make_position(
                position=math.nan,
                velocity=0.0,
                velocity_limit=self.max_velocity,
                accel_limit=self.accel_limit,
                maximum_torque=self.maximum_torque,
                watchdog_timeout=self.joy_timeout,
                query=motor_id in (9, 10, 11),
            )
            for joint_name, motor_id in JOINT_MOTOR_IDS.items()
        ]
        results = cycle_moteus_sync(self.transport, commands, timeout_s=1.0)
        positions = {
            result.id: result.values[moteus.Register.POSITION]
            for result in results
            if result is not None and result.id in (9, 10, 11)
        }
        missing = [motor_id for motor_id in (9, 10, 11) if motor_id not in positions]
        if missing:
            raise RuntimeError(f"No response from Moteus IDs: {missing}")
        return positions

    def capture_joint_positions(self, _request, response):
        if any(abs(self.velocities[name]) > 1e-6 for name in JOINT_MOTOR_IDS):
            response.success = False
            response.message = "Center the controller before capturing joint positions."
            return response

        try:
            positions = self.query_positions()
        except Exception as exc:
            response.success = False
            response.message = f"Capture failed: {type(exc).__name__}: {exc}"
            return response

        response.success = True
        response.message = (
            "motor_revolutions: "
            f"9={positions[9]:.9f}, 10={positions[10]:.9f}, 11={positions[11]:.9f}"
        )
        self.get_logger().info(f"Captured {response.message}")
        return response

    def set_runtime_a(self, _request, response):
        if self.replay_targets:
            response.success = False
            response.message = "Cannot set A while replay is active."
            return response
        if any(abs(self.velocities[name]) > 1e-6 for name in JOINT_MOTOR_IDS):
            response.success = False
            response.message = "Center the controller before setting A."
            return response
        try:
            self.runtime_a = self.query_positions()
        except Exception as exc:
            response.success = False
            response.message = f"Failed to set runtime A: {type(exc).__name__}: {exc}"
            return response
        self.position_hold_target = dict(self.runtime_a)
        response.success = True
        response.message = (
            f"runtime_A: 9={self.runtime_a[9]:.9f}, "
            f"10={self.runtime_a[10]:.9f}, 11={self.runtime_a[11]:.9f}"
        )
        self.get_logger().info(response.message)
        return response

    def start_cmr_replay(self, _request, response):
        if self.replay_targets:
            response.success = False
            response.message = "CMR replay is already active."
            return response
        if any(abs(self.velocities[name]) > 1e-6 for name in JOINT_MOTOR_IDS):
            response.success = False
            response.message = "Center the controller before starting replay."
            return response

        try:
            current_positions = self.query_positions()
        except Exception as exc:
            response.success = False
            response.message = f"Replay start failed: {type(exc).__name__}: {exc}"
            return response

        taught_a = self.taught_positions["A"]
        if self.runtime_a is None:
            offset = {motor_id: 0.0 for motor_id in (9, 10, 11)}
            reference_description = "saved absolute encoder positions"
        else:
            offset = {
                motor_id: self.runtime_a[motor_id] - taught_a[motor_id]
                for motor_id in (9, 10, 11)
            }
            reference_description = "runtime A offset"
        self.replay_names = list(REPLAY_SEQUENCE)
        self.replay_targets = [
            {
                motor_id: self.taught_positions[name][motor_id] + offset[motor_id]
                for motor_id in (9, 10, 11)
            }
            for name in self.replay_names
        ]
        self.replay_index = 0
        self.position_hold_target = None
        self.set_replay_deadline(current_positions, self.replay_targets[0])
        self.get_logger().info(f"Replay target: {self.replay_names[0]}")
        response.success = True
        response.message = f"CMR replay started using {reference_description}."
        return response

    def go_to_taught_a(self, _request, response):
        if self.replay_targets:
            response.success = False
            response.message = "Cannot move to A while CMR replay is active."
            return response
        if any(abs(self.velocities[name]) > 1e-6 for name in JOINT_MOTOR_IDS):
            response.success = False
            response.message = "Center the controller before moving to taught A."
            return response

        self.runtime_a = None
        self.position_hold_target = dict(self.taught_positions["A"])
        response.success = True
        response.message = (
            "Moving to taught A: "
            f"9={self.position_hold_target[9]:.9f}, "
            f"10={self.position_hold_target[10]:.9f}, "
            f"11={self.position_hold_target[11]:.9f}"
        )
        self.get_logger().warning(response.message)
        return response

    def cancel_replay(self, _request, response):
        was_active = bool(self.replay_targets)
        self.replay_targets = []
        self.replay_names = []
        self.replay_index = -1
        self.position_hold_target = None
        response.success = True
        response.message = "Replay cancelled." if was_active else "Replay was not active."
        self.get_logger().warning(response.message)
        return response

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
