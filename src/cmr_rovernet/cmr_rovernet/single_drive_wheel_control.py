#!/usr/bin/env python3

from __future__ import annotations

import asyncio
import time
from pathlib import Path

import rclpy
import toml
from cmr_msgs.msg import ControllerReading
from cmr_rovernet.moteus_drive_gui import make_transport_and_controllers, stop_compat
from cmr_rovernet.usama_control_testing import DRIVE_IDS
from rclpy.node import Node

import math


EXPANDED_BUTTONS = {
    "square": 4,
    "x": 5,
    "circle": 6,
    "triangle": 7,
}

LEGACY_BUTTON_VALUES = {
    "square": 1,
    "x": 256,
    "circle": 65536,
    "triangle": 16777216,
}

BUTTON_TO_MOTOR = {
    "triangle": 3,  # front right drive
    "circle": 4,    # back right drive
    "x": 2,         # back left drive
    "square": 1,    # front left drive
}

MOTOR_LABELS = {
    1: "front left",
    2: "back left",
    3: "front right",
    4: "back right",
}

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


async def command_one_drive_motor_async(
    port_arg: str,
    timeout_s: float,
    motor_id: int,
    drive_rps: float,
    max_torque: float,
    acceleration_limit: float,
    watchdog_timeout: float,
):
    port, controllers = make_transport_and_controllers(port_arg, DRIVE_IDS)
    _label, ctrl = controllers[motor_id]
    signed_velocity = DRIVE_FORWARD_SIGN[motor_id] * drive_rps
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
    return port, signed_velocity


class SingleDriveWheelControlNode(Node):
    def __init__(self) -> None:
        super().__init__("single_drive_wheel_control_node")

        self.declare_parameter("config_path", "")
        config = self._load_node_config()

        self.declare_parameter("can_port", "/dev/ttyACM0")
        self.declare_parameter("timeout_s", 0.5)
        self.declare_parameter("drive_rps", 1.0)
        self.declare_parameter("drive_max_torque", 0.5)
        self.declare_parameter("drive_acceleration_limit", 5.0)
        self.declare_parameter("watchdog_timeout_s", 0.5)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("refresh_rate_hz", 10.0)

        self.port = str(self._setting("can_port", config))
        self.timeout_s = float(self._setting("timeout_s", config))
        self.drive_rps = abs(float(self._setting("drive_rps", config)))
        self.drive_max_torque = abs(float(self._setting("drive_max_torque", config)))
        self.drive_acceleration_limit = abs(
            float(self._setting("drive_acceleration_limit", config))
        )
        self.watchdog_timeout_s = max(
            0.25,
            float(self._setting("watchdog_timeout_s", config)),
        )
        self.command_timeout_s = float(self._setting("command_timeout_s", config))
        self.refresh_rate_hz = float(self._setting("refresh_rate_hz", config))

        self._selected_button: str | None = None
        self._last_cmd_at = 0.0
        self._last_motor_id: int | None = None

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
            f"single_drive_wheel_control_node started on {self.port}: "
            f"drive_rps={self.drive_rps:.2f}, torque={self.drive_max_torque:.2f}. "
            "Triangle=front right, Circle=back right, X=back left, Square=front left."
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
        return node_config

    def _setting(self, name: str, config: dict[str, object]) -> object:
        if name in config:
            return config[name]
        return self.get_parameter(name).value

    def _controller_buttons_cb(self, msg: ControllerReading) -> None:
        pressed = self._pressed_face_buttons(list(msg.button_array))
        self._last_cmd_at = time.time()

        if len(pressed) == 1:
            self._selected_button = pressed[0]
            motor_id = BUTTON_TO_MOTOR[self._selected_button]
            if motor_id != self._last_motor_id:
                self.get_logger().info(
                    f"{self._selected_button} pressed -> motor {motor_id} "
                    f"({MOTOR_LABELS[motor_id]}) forward"
                )
            self._last_motor_id = motor_id
        else:
            if len(pressed) > 1:
                self.get_logger().warn(
                    f"Multiple wheel buttons pressed {pressed}; stopping drive motors",
                    throttle_duration_sec=1.0,
                )
            self._selected_button = None
            self._last_motor_id = None

    def _pressed_face_buttons(self, buttons: list[int]) -> list[str]:
        if len(buttons) >= 8:
            return [
                name
                for name, index in EXPANDED_BUTTONS.items()
                if int(buttons[index]) != 0
            ]

        if len(buttons) >= 2:
            button_val = int(buttons[1])
            return [
                name
                for name, value in LEGACY_BUTTON_VALUES.items()
                if button_val == value
            ]

        return []

    def _timer_cb(self) -> None:
        if self._selected_button is None or time.time() - self._last_cmd_at > self.command_timeout_s:
            asyncio.run(stop_drive_motors_async(self.port, self.timeout_s))
            return

        motor_id = BUTTON_TO_MOTOR[self._selected_button]
        try:
            _port, signed_velocity = asyncio.run(
                command_one_drive_motor_async(
                    port_arg=self.port,
                    timeout_s=self.timeout_s,
                    motor_id=motor_id,
                    drive_rps=self.drive_rps,
                    max_torque=self.drive_max_torque,
                    acceleration_limit=self.drive_acceleration_limit,
                    watchdog_timeout=self.watchdog_timeout_s,
                )
            )
            self.get_logger().info(
                f"motor {motor_id} ({MOTOR_LABELS[motor_id]}) velocity={signed_velocity:+.3f}",
                throttle_duration_sec=1.0,
            )
        except Exception as exc:
            self.get_logger().error(
                f"Single-wheel command failed: {exc!r}",
                throttle_duration_sec=1.0,
            )

    def destroy_node(self):
        self.get_logger().info("Shutting down single_drive_wheel_control_node")
        try:
            asyncio.run(stop_drive_motors_async(self.port, self.timeout_s))
        except Exception as exc:
            self.get_logger().error(f"Single-wheel shutdown stop failed: {exc!r}")
        super().destroy_node()


def ros_main(args=None):
    rclpy.init(args=args)
    node = SingleDriveWheelControlNode()
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
