#!/usr/bin/env python3

from __future__ import annotations

import asyncio
import time
from pathlib import Path

import rclpy
import toml
from cmr_msgs.msg import ControllerReading
from cmr_rovernet.moteus_drive_gui import make_transport_and_controllers, stop_compat
from cmr_rovernet.usama_control_testing import (
    COMMAND_EPSILON,
    DOUBLE_SPEED_MULTIPLIER,
    DRIVE_IDS,
    HALF_SPEED_MULTIPLIER,
    STICK_RANGE,
    TRIPLE_SPEED_MULTIPLIER,
    TRIGGER_DEADBAND,
    TRIGGER_PRESSED_THRESHOLD,
    UsamaControlRosNode,
)
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

import math


DRIVE_FORWARD_SIGN = {
    1: 1.0,   # front left
    2: 1.0,   # back left
    3: -1.0,  # front right
    4: -1.0,  # back right
}


async def stop_drive_motors_async(port_arg: str, timeout_s: float):
    port, controllers = make_transport_and_controllers(port_arg, DRIVE_IDS)
    statuses = {}
    for motor_id in DRIVE_IDS:
        _label, ctrl = controllers[motor_id]
        try:
            statuses[motor_id] = await asyncio.wait_for(
                stop_compat(ctrl),
                timeout=timeout_s,
            )
        except Exception as exc:
            statuses[motor_id] = repr(exc)
    return port, statuses


async def command_drive_motors_async(
    port_arg: str,
    timeout_s: float,
    drive_rps: float,
    max_torque: float,
    acceleration_limit: float,
    watchdog_timeout: float,
):
    port, controllers = make_transport_and_controllers(port_arg, DRIVE_IDS)
    commands: dict[int, float] = {}
    for motor_id in DRIVE_IDS:
        _label, ctrl = controllers[motor_id]
        signed_velocity = DRIVE_FORWARD_SIGN[motor_id] * drive_rps
        commands[motor_id] = signed_velocity
        await asyncio.wait_for(
            ctrl.set_position(
                position=math.nan,
                velocity=signed_velocity,
                maximum_torque=max_torque,
                accel_limit=acceleration_limit,
                watchdog_timeout=max(0.25, watchdog_timeout),
            ),
            timeout=timeout_s,
        )
    return port, commands


class DriveOnlyControlNode(Node):
    def __init__(self) -> None:
        super().__init__("drive_only_control_node")

        self.declare_parameter("config_path", "")
        config = self._load_node_config()

        self.declare_parameter("can_port", "/dev/ttyACM0")
        self.declare_parameter("timeout_s", 0.30)
        self.declare_parameter("drive_rps", 2.0)
        self.declare_parameter("half_speed_multiplier", HALF_SPEED_MULTIPLIER)
        self.declare_parameter("double_speed_multiplier", DOUBLE_SPEED_MULTIPLIER)
        self.declare_parameter("triple_speed_multiplier", TRIPLE_SPEED_MULTIPLIER)
        self.declare_parameter("trigger_pressed_threshold", TRIGGER_PRESSED_THRESHOLD)
        self.declare_parameter("drive_max_torque", 0.5)
        self.declare_parameter("drive_acceleration_limit", 5.0)
        self.declare_parameter("watchdog_timeout_s", 0.5)
        self.declare_parameter("controller_deadzone", 0.1)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("refresh_rate_hz", 10.0)

        self.port = str(self._setting("can_port", config))
        self.timeout_s = float(self._setting("timeout_s", config))
        self.drive_rps = abs(float(self._setting("drive_rps", config)))
        self.half_speed_multiplier = abs(
            float(self._setting("half_speed_multiplier", config))
        )
        self.double_speed_multiplier = abs(
            float(self._setting("double_speed_multiplier", config))
        )
        self.triple_speed_multiplier = abs(
            float(self._setting("triple_speed_multiplier", config))
        )
        self.trigger_pressed_threshold = int(
            self._setting("trigger_pressed_threshold", config)
        )
        self.drive_max_torque = abs(float(self._setting("drive_max_torque", config)))
        self.drive_acceleration_limit = abs(
            float(self._setting("drive_acceleration_limit", config))
        )
        self.watchdog_timeout_s = max(
            0.25,
            float(self._setting("watchdog_timeout_s", config)),
        )
        self.controller_deadzone = float(self._setting("controller_deadzone", config))
        self.command_timeout_s = float(self._setting("command_timeout_s", config))
        self.refresh_rate_hz = float(self._setting("refresh_rate_hz", config))

        self._drive_axis = 0.0
        self._speed_rps = 0.0
        self._last_cmd_at = 0.0
        self._estop_latched = False

        self.create_subscription(
            TwistStamped,
            "/drives_controller/cmd_vel",
            self._controller_cmd_vel_cb,
            10,
        )
        self.create_subscription(
            ControllerReading,
            "/drives_controller/cmd_buttons",
            self._controller_buttons_cb,
            10,
        )

        asyncio.run(stop_drive_motors_async(self.port, self.timeout_s))
        self._timer = self.create_timer(
            1.0 / max(self.refresh_rate_hz, 1.0),
            self._timer_cb,
        )

        self.get_logger().info(
            f"drive_only_control_node started on {self.port}: "
            f"drive_rps={self.drive_rps:.2f}, "
            f"drive_torque={self.drive_max_torque:.2f}, "
            f"drive_accel={self.drive_acceleration_limit:.2f}. "
            "Steer motors are not commanded by this node."
        )

    def _load_node_config(self) -> dict[str, object]:
        config_path = str(self.get_parameter("config_path").value)
        if not config_path:
            return {}

        path = Path(config_path).expanduser()
        if not path.exists():
            self.get_logger().warn(f"config_path does not exist: {path}")
            return {}

        try:
            data = toml.load(path)
        except Exception as exc:
            self.get_logger().warn(f"Failed to load config_path {path}: {exc!r}")
            return {}

        node_config = data.get("node", {})
        if not isinstance(node_config, dict):
            self.get_logger().warn(f"config_path {path} has no valid [node] table")
            return {}

        self.get_logger().info(f"Loaded node config from {path}")
        return node_config

    def _setting(self, name: str, config: dict[str, object]) -> object:
        if name in config:
            return config[name]
        return self.get_parameter(name).value

    def _controller_cmd_vel_cb(self, msg: TwistStamped) -> None:
        self._drive_axis = self._clamp(
            self._apply_deadzone(-msg.twist.linear.y / STICK_RANGE)
        )
        self._last_cmd_at = time.time()

        self.get_logger().info(
            f"drive-only axis={self._drive_axis:+.3f} speed_rps={self._speed_rps:+.3f}",
            throttle_duration_sec=1.0,
        )

    def _controller_buttons_cb(self, msg: ControllerReading) -> None:
        decoded = UsamaControlRosNode._decode_controller_buttons(list(msg.button_array))
        if decoded is None:
            return

        estop_pressed = bool(decoded["l1"]) and bool(decoded["triangle"])
        if estop_pressed and not self._estop_latched:
            self._estop_latched = True
            self._speed_rps = 0.0
            self._drive_axis = 0.0
            self.get_logger().warn("Drive-only estop latched from controller buttons")
            return

        if self._estop_latched and not bool(decoded["l1"]) and not bool(
            decoded["triangle"]
        ):
            self._estop_latched = False
            self.get_logger().info("Drive-only estop released")

        if self._estop_latched:
            self._speed_rps = 0.0
            self._last_cmd_at = time.time()
            return

        self._speed_rps = self._manual_speed_rps(
            l1=int(decoded["l1"]),
            r1=int(decoded["r1"]),
            l2=int(decoded["l2"]),
            r2=int(decoded["r2"]),
        )
        self._last_cmd_at = time.time()

    def _timer_cb(self) -> None:
        if self._estop_latched or time.time() - self._last_cmd_at > self.command_timeout_s:
            asyncio.run(stop_drive_motors_async(self.port, self.timeout_s))
            return

        if (
            abs(self._drive_axis) <= self.controller_deadzone
            or abs(self._speed_rps) <= COMMAND_EPSILON
        ):
            asyncio.run(stop_drive_motors_async(self.port, self.timeout_s))
            return

        drive_command_rps = self._speed_rps * (1.0 if self._drive_axis >= 0 else -1.0)
        try:
            _port, commands = asyncio.run(
                command_drive_motors_async(
                    port_arg=self.port,
                    timeout_s=self.timeout_s,
                    drive_rps=drive_command_rps,
                    max_torque=self.drive_max_torque,
                    acceleration_limit=self.drive_acceleration_limit,
                    watchdog_timeout=self.watchdog_timeout_s,
                )
            )
            self.get_logger().info(
                f"drive-only commands={commands}",
                throttle_duration_sec=1.0,
            )
        except Exception as exc:
            self.get_logger().error(
                f"Drive-only command failed: {exc!r}",
                throttle_duration_sec=1.0,
            )

    def _manual_speed_rps(self, *, l1: int, r1: int, l2: int, r2: int) -> float:
        if bool(l1):
            return self.drive_rps * self.triple_speed_multiplier
        if bool(r1):
            return -self.drive_rps
        if self._trigger_pressed(r2):
            return self.drive_rps
        if self._trigger_pressed(l2):
            return self.drive_rps * self.half_speed_multiplier
        return 0.0

    def _trigger_pressed(self, trigger_byte: int) -> bool:
        value = int(trigger_byte)
        return value >= max(TRIGGER_DEADBAND, self.trigger_pressed_threshold)

    def destroy_node(self):
        self.get_logger().info("Shutting down drive_only_control_node")
        try:
            asyncio.run(stop_drive_motors_async(self.port, self.timeout_s))
        except Exception as exc:
            self.get_logger().error(f"Drive-only shutdown stop failed: {exc!r}")
        super().destroy_node()

    def _apply_deadzone(self, value: float) -> float:
        return 0.0 if abs(value) < self.controller_deadzone else value

    @staticmethod
    def _clamp(value: float, limit: float = 1.0) -> float:
        return max(-limit, min(limit, value))


def ros_main(args=None):
    rclpy.init(args=args)
    node = DriveOnlyControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    ros_main()
