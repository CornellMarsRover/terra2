#!/usr/bin/env python3

from __future__ import annotations

import asyncio
import time
from pathlib import Path

import rclpy
import toml
from cmr_msgs.msg import ControllerReading
from cmr_rovernet.moteus_drive_gui import (
    make_transport_and_controllers,
    query_one,
    read_value,
    stop_compat,
)
from cmr_rovernet.usama_control_testing import (
    COMMAND_EPSILON,
    ROVER_LENGTH,
    ROVER_WIDTH,
    STEER_CENTER_OFFSETS,
    STEER_DEGREES_TO_POSITION,
    STEER_IDS,
    STEER_SIGN,
    STICK_RANGE,
    UsamaControlRosNode,
)
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

import math


async def query_steer_motors_async(port_arg: str, timeout_s: float):
    port, controllers = make_transport_and_controllers(port_arg, STEER_IDS)
    results = []
    for motor_id in STEER_IDS:
        label, ctrl = controllers[motor_id]
        results.append(await query_one(ctrl, motor_id, label, timeout_s))
    return port, results


async def stop_steer_motors_async(port_arg: str, timeout_s: float):
    port, controllers = make_transport_and_controllers(port_arg, STEER_IDS)
    statuses = {}
    for motor_id in STEER_IDS:
        _label, ctrl = controllers[motor_id]
        try:
            statuses[motor_id] = await asyncio.wait_for(
                stop_compat(ctrl),
                timeout=timeout_s,
            )
        except Exception as exc:
            statuses[motor_id] = repr(exc)
    return port, statuses


def calculate_steer_targets(
    vx: float,
    vy: float,
    omega: float,
    center_offsets: dict[int, float],
) -> dict[int, float]:
    radius = math.sqrt(ROVER_LENGTH**2 + ROVER_WIDTH**2)

    vx = round(vx, 3)
    vy = round(vy, 3)
    omega = round(omega, 3)

    a = vy - omega * (ROVER_LENGTH / radius)
    b = vy + omega * (ROVER_LENGTH / radius)
    c = vx - omega * (ROVER_WIDTH / radius)
    d = vx + omega * (ROVER_WIDTH / radius)

    wa1 = round(math.atan2(b, c) * 180.0 / math.pi, 3) * STEER_DEGREES_TO_POSITION
    wa2 = round(math.atan2(b, d) * 180.0 / math.pi, 3) * STEER_DEGREES_TO_POSITION
    wa3 = round(math.atan2(a, d) * 180.0 / math.pi, 3) * STEER_DEGREES_TO_POSITION
    wa4 = round(math.atan2(a, c) * 180.0 / math.pi, 3) * STEER_DEGREES_TO_POSITION

    return {
        5: center_offsets[5] + STEER_SIGN[5] * wa2,
        7: center_offsets[7] + STEER_SIGN[7] * wa1,
        6: center_offsets[6] + STEER_SIGN[6] * wa3,
        8: center_offsets[8] + STEER_SIGN[8] * wa4,
    }


async def command_steer_motors_async(
    port_arg: str,
    timeout_s: float,
    vx: float,
    vy: float,
    omega: float,
    steer_torque: float,
    steer_velocity_limit: float,
    watchdog_timeout: float,
    center_offsets: dict[int, float],
):
    port, controllers = make_transport_and_controllers(port_arg, STEER_IDS)
    targets = calculate_steer_targets(vx, vy, omega, center_offsets)

    for motor_id in STEER_IDS:
        _label, ctrl = controllers[motor_id]
        await asyncio.wait_for(
            ctrl.set_position(
                position=targets[motor_id],
                velocity_limit=steer_velocity_limit,
                maximum_torque=steer_torque,
                watchdog_timeout=max(0.25, watchdog_timeout),
            ),
            timeout=timeout_s,
        )

    return port, targets


class SteerOnlyControlNode(Node):
    def __init__(self) -> None:
        super().__init__("steer_only_control_node")

        self.declare_parameter("config_path", "")
        config = self._load_node_config()

        self.declare_parameter("can_port", "/dev/ttyACM0")
        self.declare_parameter("timeout_s", 0.30)
        self.declare_parameter("steer_max_torque", 0.5)
        self.declare_parameter("steer_velocity_limit", 1.5)
        self.declare_parameter("watchdog_timeout_s", 0.5)
        self.declare_parameter("controller_deadzone", 0.1)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("refresh_rate_hz", 10.0)
        self.declare_parameter("capture_steer_zero_on_start", True)

        self.port = str(self._setting("can_port", config))
        self.timeout_s = float(self._setting("timeout_s", config))
        self.steer_max_torque = abs(float(self._setting("steer_max_torque", config)))
        self.steer_velocity_limit = abs(
            float(self._setting("steer_velocity_limit", config))
        )
        self.watchdog_timeout_s = max(
            0.25,
            float(self._setting("watchdog_timeout_s", config)),
        )
        self.controller_deadzone = float(self._setting("controller_deadzone", config))
        self.command_timeout_s = float(self._setting("command_timeout_s", config))
        self.refresh_rate_hz = float(self._setting("refresh_rate_hz", config))
        self.capture_steer_zero_on_start = bool(
            self._setting("capture_steer_zero_on_start", config)
        )

        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        self._last_cmd_at = 0.0
        self._estop_latched = False
        self._steer_center_offsets = dict(STEER_CENTER_OFFSETS)

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

        self._initialize_hardware_state()
        self._timer = self.create_timer(
            1.0 / max(self.refresh_rate_hz, 1.0),
            self._timer_cb,
        )

        self.get_logger().info(
            f"steer_only_control_node started on {self.port}: "
            f"steer_velocity_limit={self.steer_velocity_limit:.2f}, "
            f"steer_torque={self.steer_max_torque:.2f}. "
            "Drive motors are not commanded by this node."
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
        self._vx = self._clamp(
            self._apply_deadzone(-msg.twist.linear.y / STICK_RANGE)
        )
        self._vy = self._clamp(
            self._apply_deadzone(msg.twist.linear.x / STICK_RANGE)
        )
        self._omega = self._clamp(
            self._apply_deadzone(msg.twist.angular.x / STICK_RANGE)
        )
        self._last_cmd_at = time.time()

        self.get_logger().info(
            f"steer-only cmd vx={self._vx:+.3f} vy={self._vy:+.3f} "
            f"omega={self._omega:+.3f}",
            throttle_duration_sec=1.0,
        )

    def _controller_buttons_cb(self, msg: ControllerReading) -> None:
        decoded = UsamaControlRosNode._decode_controller_buttons(list(msg.button_array))
        if decoded is None:
            return

        estop_pressed = bool(decoded["l1"]) and bool(decoded["triangle"])
        if estop_pressed and not self._estop_latched:
            self._estop_latched = True
            self._vx = self._vy = self._omega = 0.0
            self.get_logger().warn("Steer-only estop latched from controller buttons")
        elif self._estop_latched and not bool(decoded["l1"]) and not bool(
            decoded["triangle"]
        ):
            self._estop_latched = False
            self.get_logger().info("Steer-only estop released")

    def _timer_cb(self) -> None:
        if self._estop_latched or time.time() - self._last_cmd_at > self.command_timeout_s:
            asyncio.run(stop_steer_motors_async(self.port, self.timeout_s))
            return

        if max(abs(self._vx), abs(self._vy), abs(self._omega)) <= COMMAND_EPSILON:
            asyncio.run(stop_steer_motors_async(self.port, self.timeout_s))
            return

        try:
            _port, targets = asyncio.run(
                command_steer_motors_async(
                    port_arg=self.port,
                    timeout_s=self.timeout_s,
                    vx=self._vx,
                    vy=self._vy,
                    omega=self._omega,
                    steer_torque=self.steer_max_torque,
                    steer_velocity_limit=self.steer_velocity_limit,
                    watchdog_timeout=self.watchdog_timeout_s,
                    center_offsets=self._steer_center_offsets,
                )
            )
            self.get_logger().info(
                f"steer-only targets={targets}",
                throttle_duration_sec=1.0,
            )
        except Exception as exc:
            self.get_logger().error(
                f"Steer-only command failed: {exc!r}",
                throttle_duration_sec=1.0,
            )

    def _initialize_hardware_state(self) -> None:
        try:
            if self.capture_steer_zero_on_start:
                port, results = asyncio.run(
                    query_steer_motors_async(self.port, self.timeout_s)
                )
                captured = []
                for result in results:
                    if not result.ok:
                        continue
                    position = read_value(result, "ABS_POSITION")
                    if position is None:
                        position = read_value(result, "POSITION")
                    if position is None:
                        continue
                    self._steer_center_offsets[result.motor_id] = float(position)
                    captured.append(f"{result.motor_id}={float(position):.3f}")
                self.get_logger().info(
                    f"Steer zero captured on {port or 'default transport'}: "
                    + (", ".join(captured) if captured else "no valid steer positions")
                )
            asyncio.run(stop_steer_motors_async(self.port, self.timeout_s))
        except Exception as exc:
            self.get_logger().error(f"Steer-only initialization error: {exc!r}")

    def destroy_node(self):
        self.get_logger().info("Shutting down steer_only_control_node")
        try:
            asyncio.run(stop_steer_motors_async(self.port, self.timeout_s))
        except Exception as exc:
            self.get_logger().error(f"Steer-only shutdown stop failed: {exc!r}")
        super().destroy_node()

    def _apply_deadzone(self, value: float) -> float:
        return 0.0 if abs(value) < self.controller_deadzone else value

    @staticmethod
    def _clamp(value: float, limit: float = 1.0) -> float:
        return max(-limit, min(limit, value))


def ros_main(args=None):
    rclpy.init(args=args)
    node = SteerOnlyControlNode()
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
