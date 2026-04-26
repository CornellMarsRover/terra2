#!/usr/bin/env python3
"""
Direct moteus GUI for straight drive bump testing.

This uses the same style of direct drive command as the moteus debug tool,
but applies it to all four drive motors together so we can verify that the
rover moves forward and backward with stronger, tunable commands.
"""

from __future__ import annotations

import asyncio
import argparse
import math
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import ttk

import rclpy
import toml
from cmr_msgs.msg import ControllerReading
from cmr_rovernet.moteus_drive_gui import (
    EXPECTED_MODULES,
    fmt,
    make_transport_and_controllers,
    query_one,
    read_value,
    stop_compat,
)
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


DRIVE_IDS = [1, 2, 3, 4]
# Actual steer motor layout on the rover:
# 5 = front left, 7 = front right, 6 = back left, 8 = back right
STEER_IDS = [5, 7, 6, 8]
STEER_DEGREES_TO_POSITION = 50.0 / 360.0
STEER_DIRECTION_SIGN = 1.0
STEER_CENTER_OFFSETS = {
    5: 0.0,
    7: 0.0,
    6: 0.0,
    8: 0.0,
}

# Left-side drives are positive for forward motion.
# Right-side drives are negative for forward motion.
FORWARD_SIGN = {
    1: 1.0,   # FL
    2: 1.0,   # BL
    3: -1.0,  # FR
    4: -1.0,  # BR
}

STEER_SIGN = {
    5: 1.0,  # FL steer
    7: 1.0,  # FR steer
    6: 1.0,  # BL steer
    8: 1.0,  # BR steer
}

BTN_L1 = 0
BTN_R1 = 1
BTN_L2 = 2
BTN_R2 = 3
BTN_TRIANGLE = 7

LEGACY_L1 = 1
LEGACY_L2_MIN = 65536
LEGACY_L2 = 16711680
LEGACY_TRIANGLE = 16777216

STICK_RANGE = 2.5
TRIGGER_DEADBAND = 5
TRIGGER_MAX = 255

ROVER_LENGTH = 39.0
ROVER_WIDTH = 39.0
COMMAND_EPSILON = 1e-3


@dataclass
class ManualCommandState:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    speed_rps: float = 0.0
    updated_at: float = 0.0


@dataclass
class AutonomyCommandState:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    updated_at: float = 0.0


async def query_drive_motors_async(
    port_arg: str,
    timeout_s: float,
):
    port, controllers = make_transport_and_controllers(port_arg, DRIVE_IDS)
    results = []
    for motor_id in DRIVE_IDS:
        label, ctrl = controllers[motor_id]
        results.append(await query_one(ctrl, motor_id, label, timeout_s))
    return port, results


async def query_all_motors_async(
    port_arg: str,
    timeout_s: float,
):
    all_ids = DRIVE_IDS + STEER_IDS
    port, controllers = make_transport_and_controllers(port_arg, all_ids)
    results = []
    for motor_id in all_ids:
        label, ctrl = controllers[motor_id]
        results.append(await query_one(ctrl, motor_id, label, timeout_s))
    return port, results


async def query_steer_motors_async(
    port_arg: str,
    timeout_s: float,
):
    port, controllers = make_transport_and_controllers(port_arg, STEER_IDS)
    results = []
    for motor_id in STEER_IDS:
        label, ctrl = controllers[motor_id]
        results.append(await query_one(ctrl, motor_id, label, timeout_s))
    return port, results


async def zero_steer_outputs_async(
    port_arg: str,
    timeout_s: float,
):
    port, controllers = make_transport_and_controllers(port_arg, STEER_IDS)
    statuses = {}
    for motor_id in STEER_IDS:
        _label, ctrl = controllers[motor_id]
        try:
            await asyncio.wait_for(
                ctrl.set_output_nearest(position=0.0),
                timeout=timeout_s,
            )
            result = await asyncio.wait_for(ctrl.query(), timeout=timeout_s)
            abs_position = read_value(result, "ABS_POSITION")
            position = read_value(result, "POSITION")
            statuses[motor_id] = {
                "abs_position": abs_position,
                "position": position,
            }
        except Exception as exc:
            statuses[motor_id] = {"error": repr(exc)}
    return port, statuses


async def stop_drive_motors_async(
    port_arg: str,
    timeout_s: float,
):
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


async def stop_all_motors_async(
    port_arg: str,
    timeout_s: float,
):
    all_ids = DRIVE_IDS + STEER_IDS
    port, controllers = make_transport_and_controllers(port_arg, all_ids)
    statuses = {}
    for motor_id in all_ids:
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
    direction: float,
    watchdog_timeout: float,
):
    port, controllers = make_transport_and_controllers(port_arg, DRIVE_IDS)
    for motor_id in DRIVE_IDS:
        _label, ctrl = controllers[motor_id]
        signed_velocity = direction * FORWARD_SIGN[motor_id] * drive_rps
        await asyncio.wait_for(
            ctrl.set_position(
                position=math.nan,
                velocity=signed_velocity,
                maximum_torque=max_torque,
                accel_limit=5.0,
                watchdog_timeout=max(0.25, watchdog_timeout),
            ),
            timeout=timeout_s,
        )
    return port


async def command_orbit_motors_async(
    port_arg: str,
    timeout_s: float,
    drive_rps: float,
    max_torque: float,
    orbit_direction: float,
    watchdog_timeout: float,
):
    port, controllers = make_transport_and_controllers(port_arg, DRIVE_IDS)
    # Orbit is defined in rover motion terms, not raw motor-sign terms:
    # left side (1,2) forward while right side (3,4) backward for one direction,
    # and the opposite for the other.
    orbit_forward_sign = {
        1: 1.0,   # left side forward
        2: 1.0,   # left side forward
        3: 1.0,   # right side backward requires positive raw motor command
        4: 1.0,   # right side backward requires positive raw motor command
    }
    for motor_id in DRIVE_IDS:
        _label, ctrl = controllers[motor_id]
        signed_velocity = orbit_direction * orbit_forward_sign[motor_id] * drive_rps
        await asyncio.wait_for(
            ctrl.set_position(
                position=math.nan,
                velocity=signed_velocity,
                maximum_torque=max_torque,
                accel_limit=5.0,
                watchdog_timeout=max(0.25, watchdog_timeout),
            ),
            timeout=timeout_s,
        )
    return port


async def command_steer_motors_async(
    port_arg: str,
    timeout_s: float,
    steer_angle_deg: float,
    max_torque: float,
    velocity_limit: float,
    watchdog_timeout: float,
    center_offsets: dict[int, float],
):
    port, controllers = make_transport_and_controllers(port_arg, STEER_IDS)
    steer_delta = steer_angle_deg * STEER_DEGREES_TO_POSITION

    for motor_id in STEER_IDS:
        _label, ctrl = controllers[motor_id]
        target_position = center_offsets[motor_id] + (
            STEER_SIGN[motor_id] * steer_delta
        )
        await asyncio.wait_for(
            ctrl.set_position(
                position=target_position,
                velocity_limit=velocity_limit,
                maximum_torque=max_torque,
                watchdog_timeout=max(0.25, watchdog_timeout),
            ),
            timeout=timeout_s,
        )
    return port


def calculate_swerve_targets(
    vx: float,
    vy: float,
    omega: float,
    drive_rps: float,
    center_offsets: dict[int, float],
) -> dict[str, dict[int, float]]:
    radius = math.sqrt(ROVER_LENGTH**2 + ROVER_WIDTH**2)

    vx = round(vx, 3)
    vy = round(vy, 3)
    omega = round(omega, 3)

    a = vy - omega * (ROVER_LENGTH / radius)
    b = vy + omega * (ROVER_LENGTH / radius)
    c = vx - omega * (ROVER_WIDTH / radius)
    d = vx + omega * (ROVER_WIDTH / radius)

    ws1 = math.sqrt(b**2 + c**2)  # back right
    ws2 = math.sqrt(b**2 + d**2)  # front left
    ws3 = math.sqrt(a**2 + d**2)  # back left
    ws4 = math.sqrt(a**2 + c**2)  # front right

    max_speed = max(ws1, ws2, ws3, ws4, 1.0)
    ws1 /= max_speed
    ws2 /= max_speed
    ws3 /= max_speed
    ws4 /= max_speed

    wa1 = (round(math.atan2(b, c) * 180.0 / math.pi, 3)) * STEER_DEGREES_TO_POSITION
    wa2 = (round(math.atan2(b, d) * 180.0 / math.pi, 3)) * STEER_DEGREES_TO_POSITION
    wa3 = (round(math.atan2(a, d) * 180.0 / math.pi, 3)) * STEER_DEGREES_TO_POSITION
    wa4 = (round(math.atan2(a, c) * 180.0 / math.pi, 3)) * STEER_DEGREES_TO_POSITION

    return {
        "drive": {
            1: FORWARD_SIGN[1] * drive_rps * ws2,
            2: FORWARD_SIGN[2] * drive_rps * ws3,
            3: FORWARD_SIGN[3] * drive_rps * ws4,
            4: FORWARD_SIGN[4] * drive_rps * ws1,
        },
        "steer": {
            5: center_offsets[5] + STEER_SIGN[5] * wa2,
            7: center_offsets[7] + STEER_SIGN[7] * wa1,
            6: center_offsets[6] + STEER_SIGN[6] * wa3,
            8: center_offsets[8] + STEER_SIGN[8] * wa4,
        },
    }


async def command_swerve_motors_async(
    port_arg: str,
    timeout_s: float,
    vx: float,
    vy: float,
    omega: float,
    drive_rps: float,
    drive_torque: float,
    steer_torque: float,
    steer_velocity_limit: float,
    watchdog_timeout: float,
    center_offsets: dict[int, float],
):
    all_ids = DRIVE_IDS + STEER_IDS
    port, controllers = make_transport_and_controllers(port_arg, all_ids)
    targets = calculate_swerve_targets(vx, vy, omega, drive_rps, center_offsets)

    for motor_id in DRIVE_IDS:
        _label, ctrl = controllers[motor_id]
        await asyncio.wait_for(
            ctrl.set_position(
                position=math.nan,
                velocity=targets["drive"][motor_id],
                maximum_torque=drive_torque,
                accel_limit=5.0,
                watchdog_timeout=max(0.25, watchdog_timeout),
            ),
            timeout=timeout_s,
        )

    for motor_id in STEER_IDS:
        _label, ctrl = controllers[motor_id]
        await asyncio.wait_for(
            ctrl.set_position(
                position=targets["steer"][motor_id],
                velocity_limit=steer_velocity_limit,
                maximum_torque=steer_torque,
                watchdog_timeout=max(0.25, watchdog_timeout),
            ),
            timeout=timeout_s,
        )

    return port, targets


class UsamaControlRosNode(Node):
    """ROS topic ingress using the working direct moteus command behavior."""

    def __init__(self) -> None:
        super().__init__("usama_control_testing_node")

        self.declare_parameter("config_path", "")
        config = self._load_node_config()

        self.declare_parameter("can_port", "/dev/ttyACM0")
        self.declare_parameter("timeout_s", 0.30)
        self.declare_parameter("drive_rps", 4.0)
        self.declare_parameter("drive_max_torque", 2.0)
        self.declare_parameter("steer_max_torque", 3.0)
        self.declare_parameter("steer_velocity_limit", 6.0)
        self.declare_parameter("watchdog_timeout_s", 0.5)
        self.declare_parameter("controller_deadzone", 0.1)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("refresh_rate_hz", 10.0)
        self.declare_parameter("manual_override_priority", True)
        self.declare_parameter("autonomy_priority", True)
        self.declare_parameter("capture_steer_zero_on_start", True)

        self.port = str(self._setting("can_port", config))
        self.timeout_s = float(self._setting("timeout_s", config))
        self.drive_rps = abs(float(self._setting("drive_rps", config)))
        self.drive_max_torque = abs(float(self._setting("drive_max_torque", config)))
        self.steer_max_torque = abs(float(self._setting("steer_max_torque", config)))
        self.steer_velocity_limit = abs(float(self._setting("steer_velocity_limit", config)))
        self.watchdog_timeout_s = max(0.25, float(self._setting("watchdog_timeout_s", config)))
        self.controller_deadzone = float(self._setting("controller_deadzone", config))
        self.command_timeout_s = float(self._setting("command_timeout_s", config))
        self.refresh_rate_hz = float(self._setting("refresh_rate_hz", config))
        self.manual_override_priority = bool(self._setting("manual_override_priority", config))
        self.autonomy_priority = bool(self._setting("autonomy_priority", config))
        self.capture_steer_zero_on_start = bool(self._setting("capture_steer_zero_on_start", config))

        self._manual = ManualCommandState()
        self._autonomy = AutonomyCommandState()
        self._estop_latched = False
        self._last_source = "idle"
        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._steer_center_offsets = dict(STEER_CENTER_OFFSETS)
        self._last_manual_drive_axis_sign = 1.0

        self.create_subscription(TwistStamped, "/drives_controller/cmd_vel", self._controller_cmd_vel_cb, 10)
        self.create_subscription(
            ControllerReading,
            "/drives_controller/cmd_buttons",
            self._controller_buttons_cb,
            10,
        )
        self.create_subscription(Twist, "/cmd_vel_drives", self._autonomy_cmd_vel_cb, 10)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "usama_control_testing_node started. Inputs: /drives_controller/cmd_vel, "
            "/drives_controller/cmd_buttons, /cmd_vel_drives"
        )
        self.get_logger().info(
            f"Using direct moteus command behavior on {self.port}: "
            f"drive_rps={self.drive_rps:.2f}, drive_torque={self.drive_max_torque:.2f}, "
            f"steer_torque={self.steer_max_torque:.2f}"
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
        vx = self._apply_deadzone(-msg.twist.linear.y / STICK_RANGE, self.controller_deadzone)
        vy = self._apply_deadzone(msg.twist.linear.x / STICK_RANGE, self.controller_deadzone)
        omega = self._apply_deadzone(msg.twist.angular.x / STICK_RANGE, self.controller_deadzone)

        with self._lock:
            self._manual.vx = self._clamp(vx)
            self._manual.vy = self._clamp(vy)
            self._manual.omega = self._clamp(omega)
            self._manual.updated_at = time.time()

        self.get_logger().info(
            f"controller cmd_vel vx={self._clamp(vx):+.3f} "
            f"vy={self._clamp(vy):+.3f} omega={self._clamp(omega):+.3f}",
            throttle_duration_sec=1.0,
        )

    def _controller_buttons_cb(self, msg: ControllerReading) -> None:
        decoded = self._decode_controller_buttons(list(msg.button_array))
        if decoded is None:
            self.get_logger().warn(
                f"/drives_controller/cmd_buttons expected 2 or 8 entries, got {len(msg.button_array)}",
                throttle_duration_sec=2.0,
            )
            return

        l1 = decoded["l1"]
        triangle = decoded["triangle"]
        l2 = decoded["l2"]
        r2 = decoded["r2"]
        estop_pressed = bool(l1) and bool(triangle)

        with self._lock:
            if estop_pressed and not self._estop_latched:
                self._estop_latched = True
                self._manual = ManualCommandState(updated_at=time.time())
                self.get_logger().warn("Drive estop latched from controller buttons")
                return

            if self._estop_latched and not bool(l1) and not bool(triangle):
                self._estop_latched = False
                self._manual = ManualCommandState(updated_at=time.time())
                self.get_logger().info("Drive estop released")

            if self._estop_latched:
                self._manual.speed_rps = 0.0
                self._manual.updated_at = time.time()
                return

            forward_rps = self._trigger_to_speed_scale(r2) * self.drive_rps
            reverse_rps = self._trigger_to_speed_scale(l2) * self.drive_rps
            if forward_rps > 0.0:
                speed_rps = forward_rps
            elif reverse_rps > 0.0:
                speed_rps = -reverse_rps
            else:
                speed_rps = 0.0

            self._manual.speed_rps = speed_rps
            self._manual.updated_at = time.time()

        self.get_logger().info(
            f"controller buttons L2={l2} R2={r2} speed_rps={speed_rps:+.3f}",
            throttle_duration_sec=1.0,
        )

    def _autonomy_cmd_vel_cb(self, msg: Twist) -> None:
        with self._lock:
            self._autonomy.vx = self._clamp(msg.linear.x)
            self._autonomy.vy = self._clamp(msg.linear.y)
            self._autonomy.omega = self._clamp(msg.angular.z)
            self._autonomy.updated_at = time.time()

        self.get_logger().info(
            f"autonomy cmd_vel vx={self._clamp(msg.linear.x):+.3f} "
            f"vy={self._clamp(msg.linear.y):+.3f} omega={self._clamp(msg.angular.z):+.3f}",
            throttle_duration_sec=1.0,
        )

    def _worker_loop(self) -> None:
        self._initialize_hardware_state()
        refresh_period = 1.0 / max(self.refresh_rate_hz, 1.0)

        while not self._shutdown.is_set():
            try:
                command = self._select_command()
                self.get_logger().info(f"selected drive command: {command}", throttle_duration_sec=1.0)
                if command["mode"] in {"idle", "estop"}:
                    if self._last_source != command["source"]:
                        self.get_logger().info(f"Drive source switched to {command['source']}")
                    asyncio.run(stop_all_motors_async(self.port, self.timeout_s))
                    self._last_source = str(command["source"])
                else:
                    _port, targets = asyncio.run(
                        command_swerve_motors_async(
                            port_arg=self.port,
                            timeout_s=self.timeout_s,
                            vx=float(command["vx"]),
                            vy=float(command["vy"]),
                            omega=float(command["omega"]),
                            drive_rps=float(command["speed_rps"]),
                            drive_torque=self.drive_max_torque,
                            steer_torque=self.steer_max_torque,
                            steer_velocity_limit=self.steer_velocity_limit,
                            watchdog_timeout=self.watchdog_timeout_s,
                            center_offsets=self._steer_center_offsets,
                        )
                    )
                    if command["source"] != self._last_source:
                        self.get_logger().info(
                            f"Drive source switched to {command['source']}; targets={targets}"
                        )
                    self._last_source = str(command["source"])
            except Exception as exc:
                self.get_logger().error(f"Drive worker error: {exc!r}", throttle_duration_sec=1.0)
            finally:
                time.sleep(refresh_period)

        try:
            asyncio.run(stop_all_motors_async(self.port, self.timeout_s))
        except Exception as exc:
            self.get_logger().error(f"Drive shutdown stop failed: {exc!r}")

    def _initialize_hardware_state(self) -> None:
        try:
            if self.capture_steer_zero_on_start:
                port, results = asyncio.run(query_steer_motors_async(self.port, self.timeout_s))
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
            asyncio.run(stop_all_motors_async(self.port, self.timeout_s))
        except Exception as exc:
            self.get_logger().error(f"Drive initialization error: {exc!r}")

    def _select_command(self) -> dict[str, object]:
        now = time.time()
        with self._lock:
            if self._estop_latched:
                return {"mode": "estop", "source": "controller_estop"}
            manual = ManualCommandState(**self._manual.__dict__)
            autonomy = AutonomyCommandState(**self._autonomy.__dict__)

        manual_active = (now - manual.updated_at) <= self.command_timeout_s and (
            abs(manual.speed_rps) > COMMAND_EPSILON
            or any(abs(value) > self.controller_deadzone for value in (manual.vx, manual.vy, manual.omega))
        )
        autonomy_active = (now - autonomy.updated_at) <= self.command_timeout_s and any(
            abs(value) > COMMAND_EPSILON for value in (autonomy.vx, autonomy.vy, autonomy.omega)
        )

        if manual_active and self.manual_override_priority:
            return self._manual_command(manual)
        if autonomy_active and self.autonomy_priority:
            return self._autonomy_command(autonomy)
        if manual_active:
            return self._manual_command(manual)
        if autonomy_active:
            return self._autonomy_command(autonomy)
        return {"mode": "idle", "source": "idle"}

    def _manual_command(self, manual: ManualCommandState) -> dict[str, object]:
        drive_axis_sign = self._manual_drive_axis_sign(manual.vx)
        speed_rps = abs(manual.speed_rps) * drive_axis_sign
        return {
            "mode": "swerve",
            "source": "controller_topics",
            "vx": abs(manual.vx),
            "vy": manual.vy,
            "omega": manual.omega,
            "speed_rps": speed_rps,
        }

    def _manual_drive_axis_sign(self, vx: float) -> float:
        if abs(vx) > self.controller_deadzone:
            self._last_manual_drive_axis_sign = 1.0 if vx >= 0.0 else -1.0
        return self._last_manual_drive_axis_sign

    def _autonomy_command(self, autonomy: AutonomyCommandState) -> dict[str, object]:
        return {
            "mode": "swerve",
            "source": "autonomy_cmd_vel",
            "vx": autonomy.vx,
            "vy": autonomy.vy,
            "omega": autonomy.omega,
            "speed_rps": self.drive_rps,
        }

    def destroy_node(self):
        self.get_logger().info("Shutting down usama_control_testing_node")
        self._shutdown.set()
        try:
            if self._worker.is_alive():
                self._worker.join(timeout=2.0)
        except RuntimeError:
            pass
        super().destroy_node()

    @staticmethod
    def _decode_controller_buttons(buttons: list[int]) -> dict[str, int | str] | None:
        if len(buttons) >= 8:
            return {
                "format": "expanded",
                "l1": int(buttons[BTN_L1]),
                "triangle": int(buttons[BTN_TRIANGLE]),
                "l2": int(buttons[BTN_L2]),
                "r2": int(buttons[BTN_R2]),
            }

        if len(buttons) >= 2:
            trigger_val = int(buttons[0])
            button_val = int(buttons[1])
            return {
                "format": "legacy_packed",
                "trigger_val": trigger_val,
                "button_val": button_val,
                "l1": int(trigger_val == LEGACY_L1),
                "triangle": int(button_val == LEGACY_TRIANGLE),
                "l2": UsamaControlRosNode._legacy_l2_value(trigger_val),
                "r2": UsamaControlRosNode._legacy_r2_value(trigger_val),
            }

        return None

    @staticmethod
    def _legacy_r2_value(trigger_val: int) -> int:
        hex_value = hex(trigger_val & 0xFFFFFFFF)
        first_two_hex = hex_value[2:4]
        result_int = int(first_two_hex, 16)
        if 0 <= result_int <= TRIGGER_MAX:
            return result_int
        return 0

    @staticmethod
    def _legacy_l2_value(trigger_val: int) -> int:
        if LEGACY_L2_MIN <= trigger_val <= LEGACY_L2:
            return int(round(((trigger_val - LEGACY_L2_MIN) / (LEGACY_L2 - LEGACY_L2_MIN)) * TRIGGER_MAX))
        return 0

    @staticmethod
    def _trigger_to_speed_scale(trigger_byte: int) -> float:
        value = int(trigger_byte)
        if value < TRIGGER_DEADBAND:
            return 0.0
        if value > TRIGGER_MAX:
            value = TRIGGER_MAX
        return value / TRIGGER_MAX

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        return 0.0 if abs(value) < deadzone else value

    @staticmethod
    def _clamp(value: float, limit: float = 1.0) -> float:
        return max(-limit, min(limit, value))


class DriveBumpGuiApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("CMR Ackermann Drive GUI")
        self.root.geometry("980x680")

        self.port_var = tk.StringVar(value="/dev/ttyACM0")
        self.timeout_var = tk.StringVar(value="0.30")
        self.drive_rps_var = tk.StringVar(value="4.0")
        self.watchdog_var = tk.StringVar(value="0.5")
        self.drive_torque_var = tk.StringVar(value="2.0")
        self.steer_angle_var = tk.StringVar(value="45.0")
        self.steer_torque_var = tk.StringVar(value="3.0")
        self.steer_vel_limit_var = tk.StringVar(value="6.0")

        self.status_var = tk.StringVar(value="Ready")
        self.control_state_var = tk.StringVar(value="Center")
        self.control_debug_var = tk.StringVar(value="keyboard/buttons ready")
        self.rows: dict[int, dict[str, tk.StringVar]] = {}
        self.task_queue: queue.Queue = queue.Queue()
        self.result_queue: queue.Queue = queue.Queue()
        self.motion_refresh_s = 0.10
        self.status_refresh_ms = 1000
        self.status_query_pending = False
        self.current_drive_direction = 0.0
        self.current_orbit_direction = 0.0
        self.current_steer_direction = 0.0
        self.current_steer_angle_deg = 0.0
        self.steer_button_direction = 0.0
        self.keyboard_pressed: set[str] = set()
        self.keyboard_poll_ms = 50
        self.keyboard_drive_action_active: str | None = None
        self.keyboard_steer_action_active: str | None = None
        self.keyboard_orbit_action_active: str | None = None
        self.current_joystick_label = "Center"
        self.current_drive_button_label = "Center"
        self.current_orbit_button_label = "Center"
        self.current_steer_button_label = "Center"
        self.steer_center_offsets = dict(STEER_CENTER_OFFSETS)
        self.direction_buttons: dict[str, ttk.Button] = {}

        self._build_layout()
        self._bind_keyboard_controls()

        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()
        self.root.after(100, self._poll_results)
        self.root.after(self.status_refresh_ms, self._periodic_query_all)
        self.root.after(self.keyboard_poll_ms, self._poll_keyboard_controls)
        self.capture_steer_zero()
        self.queue_stop_all()
        self.queue_query_all()

    def _build_layout(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        conn = ttk.LabelFrame(outer, text="Connection", padding=8)
        conn.pack(fill="x")

        ttk.Label(conn, text="Port").grid(row=0, column=0, sticky="w")
        ttk.Entry(conn, textvariable=self.port_var, width=24).grid(
            row=0,
            column=1,
            padx=4,
            sticky="w",
        )
        ttk.Label(conn, text="Timeout (s)").grid(row=0, column=2, sticky="w", padx=(12, 0))
        ttk.Entry(conn, textvariable=self.timeout_var, width=8).grid(
            row=0,
            column=3,
            padx=4,
            sticky="w",
        )

        drive_settings = ttk.LabelFrame(outer, text="Drive Settings", padding=8)
        drive_settings.pack(fill="x", pady=(10, 0))

        ttk.Label(drive_settings, text="Speed (rev/s)").grid(row=0, column=0, sticky="w")
        ttk.Entry(drive_settings, textvariable=self.drive_rps_var, width=10).grid(
            row=0,
            column=1,
            padx=4,
            sticky="w",
        )
        ttk.Label(drive_settings, text="Watchdog (s)").grid(row=0, column=2, sticky="w", padx=(12, 0))
        ttk.Entry(drive_settings, textvariable=self.watchdog_var, width=10).grid(
            row=0,
            column=3,
            padx=4,
            sticky="w",
        )
        ttk.Label(drive_settings, text="Max Torque").grid(row=0, column=4, sticky="w", padx=(12, 0))
        ttk.Entry(drive_settings, textvariable=self.drive_torque_var, width=10).grid(
            row=0,
            column=5,
            padx=4,
            sticky="w",
        )

        steer_settings = ttk.LabelFrame(outer, text="Steer Settings", padding=8)
        steer_settings.pack(fill="x", pady=(10, 0))

        ttk.Label(steer_settings, text="Angle (deg)").grid(row=0, column=0, sticky="w")
        ttk.Entry(steer_settings, textvariable=self.steer_angle_var, width=10).grid(
            row=0,
            column=1,
            padx=4,
            sticky="w",
        )
        ttk.Label(steer_settings, text="Vel Limit").grid(row=0, column=2, sticky="w", padx=(12, 0))
        ttk.Entry(steer_settings, textvariable=self.steer_vel_limit_var, width=10).grid(
            row=0,
            column=3,
            padx=4,
            sticky="w",
        )
        ttk.Label(steer_settings, text="Max Torque").grid(row=0, column=4, sticky="w", padx=(12, 0))
        ttk.Entry(steer_settings, textvariable=self.steer_torque_var, width=10).grid(
            row=0,
            column=5,
            padx=4,
            sticky="w",
        )

        control_row = ttk.Frame(outer, padding=(0, 10, 0, 0))
        control_row.pack(fill="x")

        util_frame = ttk.LabelFrame(control_row, text="Controls", padding=8)
        util_frame.pack(side="left", fill="y")

        ttk.Label(util_frame, text="Control State").pack(anchor="w")
        ttk.Label(util_frame, textvariable=self.control_state_var).pack(anchor="w", pady=(0, 12))
        ttk.Label(util_frame, text="Control Debug").pack(anchor="w")
        ttk.Label(util_frame, textvariable=self.control_debug_var, wraplength=220).pack(anchor="w", pady=(0, 12))
        ttk.Button(util_frame, text="Center Steering", command=self.center_steering).pack(
            fill="x",
            pady=(0, 8),
        )
        ttk.Button(util_frame, text="Stop All", command=self.queue_stop_all).pack(
            fill="x",
            pady=(0, 8),
        )
        ttk.Button(util_frame, text="Query Drives", command=self.queue_query_drives).pack(
            fill="x",
        )

        buttons_frame = ttk.LabelFrame(control_row, text="Direction Toggles", padding=8)
        buttons_frame.pack(side="left", fill="y", padx=(0, 10))

        self.direction_buttons["North"] = ttk.Button(
            buttons_frame,
            text="North",
            command=lambda: self.toggle_latched_direction("North", 1.0, 0.0),
        )
        self.direction_buttons["North"].grid(row=0, column=1, padx=4, pady=4, sticky="ew")

        self.direction_buttons["West"] = ttk.Button(
            buttons_frame,
            text="West",
        )
        self.direction_buttons["West"].grid(row=1, column=0, padx=4, pady=4, sticky="ew")
        self.direction_buttons["West"].bind(
            "<ButtonPress-1>",
            lambda _event: self._on_steer_button_press("West", -1.0),
        )
        self.direction_buttons["West"].bind(
            "<ButtonRelease-1>",
            lambda _event: self._on_steer_button_release("West"),
        )

        self.direction_buttons["East"] = ttk.Button(
            buttons_frame,
            text="East",
        )
        self.direction_buttons["East"].grid(row=1, column=2, padx=4, pady=4, sticky="ew")
        self.direction_buttons["East"].bind(
            "<ButtonPress-1>",
            lambda _event: self._on_steer_button_press("East", 1.0),
        )
        self.direction_buttons["East"].bind(
            "<ButtonRelease-1>",
            lambda _event: self._on_steer_button_release("East"),
        )

        self.direction_buttons["South"] = ttk.Button(
            buttons_frame,
            text="South",
            command=lambda: self.toggle_latched_direction("South", -1.0, 0.0),
        )
        self.direction_buttons["South"].grid(row=2, column=1, padx=4, pady=4, sticky="ew")

        self.direction_buttons["Orbit West"] = ttk.Button(
            buttons_frame,
            text="Orbit West",
        )
        self.direction_buttons["Orbit West"].grid(row=3, column=0, padx=4, pady=4, sticky="ew")
        self.direction_buttons["Orbit West"].bind(
            "<ButtonPress-1>",
            lambda _event: self._on_orbit_button_press("Orbit West", -1.0),
        )
        self.direction_buttons["Orbit West"].bind(
            "<ButtonRelease-1>",
            lambda _event: self._on_orbit_button_release("Orbit West"),
        )

        self.direction_buttons["Orbit East"] = ttk.Button(
            buttons_frame,
            text="Orbit East",
        )
        self.direction_buttons["Orbit East"].grid(row=3, column=2, padx=4, pady=4, sticky="ew")
        self.direction_buttons["Orbit East"].bind(
            "<ButtonPress-1>",
            lambda _event: self._on_orbit_button_press("Orbit East", 1.0),
        )
        self.direction_buttons["Orbit East"].bind(
            "<ButtonRelease-1>",
            lambda _event: self._on_orbit_button_release("Orbit East"),
        )

        ttk.Button(buttons_frame, text="Center", command=self.reset_direction_toggle).grid(
            row=1,
            column=1,
            padx=4,
            pady=4,
            sticky="ew",
        )

        for col in range(3):
            buttons_frame.columnconfigure(col, weight=1)

        ttk.Label(
            outer,
            text="Use buttons or keyboard: W=North, S=South, A=West, D=East, J=Orbit East, L=Orbit West.",
            padding=(0, 10, 0, 4),
        ).pack(anchor="w")

        main_content = ttk.Frame(outer)
        main_content.pack(fill="both", expand=True, pady=(10, 0))

        left_panel = ttk.Frame(main_content)
        left_panel.pack(side="left", fill="both", expand=True)

        right_panel = ttk.LabelFrame(main_content, text="Motor Runtime Status", padding=8)
        right_panel.pack(side="left", fill="y", padx=(10, 0))

        table = ttk.LabelFrame(left_panel, text="Drive Motor Status", padding=8)
        table.pack(fill="x")

        headers = ["ID", "Label", "State", "Mode", "Fault", "Position", "Velocity", "Torque"]
        for col, header in enumerate(headers):
            ttk.Label(table, text=header).grid(row=0, column=col, padx=6, pady=2, sticky="w")

        for row_index, motor_id in enumerate(DRIVE_IDS, start=1):
            vars_for_row = {
                "state": tk.StringVar(value="-"),
                "mode": tk.StringVar(value="-"),
                "fault": tk.StringVar(value="-"),
                "position": tk.StringVar(value="-"),
                "velocity": tk.StringVar(value="-"),
                "torque": tk.StringVar(value="-"),
            }
            self.rows[motor_id] = vars_for_row

            ttk.Label(table, text=str(motor_id)).grid(row=row_index, column=0, padx=6, pady=2, sticky="w")
            ttk.Label(table, text=EXPECTED_MODULES[motor_id]).grid(
                row=row_index,
                column=1,
                padx=6,
                pady=2,
                sticky="w",
            )
            ttk.Label(table, textvariable=vars_for_row["state"]).grid(row=row_index, column=2, padx=6, pady=2, sticky="w")
            ttk.Label(table, textvariable=vars_for_row["mode"]).grid(row=row_index, column=3, padx=6, pady=2, sticky="w")
            ttk.Label(table, textvariable=vars_for_row["fault"]).grid(row=row_index, column=4, padx=6, pady=2, sticky="w")
            ttk.Label(table, textvariable=vars_for_row["position"]).grid(row=row_index, column=5, padx=6, pady=2, sticky="w")
            ttk.Label(table, textvariable=vars_for_row["velocity"]).grid(row=row_index, column=6, padx=6, pady=2, sticky="w")
            ttk.Label(table, textvariable=vars_for_row["torque"]).grid(row=row_index, column=7, padx=6, pady=2, sticky="w")

        runtime_headers = [
            "ID",
            "Label",
            "State",
            "Mode",
            "Fault",
            "Position",
            "Velocity",
            "Torque",
        ]
        for col, header in enumerate(runtime_headers):
            ttk.Label(right_panel, text=header).grid(row=0, column=col, padx=4, pady=2, sticky="w")

        for row_index, motor_id in enumerate(DRIVE_IDS + STEER_IDS, start=1):
            vars_for_row = self.rows.get(motor_id)
            if vars_for_row is None:
                vars_for_row = {
                    "state": tk.StringVar(value="-"),
                    "mode": tk.StringVar(value="-"),
                    "fault": tk.StringVar(value="-"),
                    "position": tk.StringVar(value="-"),
                    "velocity": tk.StringVar(value="-"),
                    "torque": tk.StringVar(value="-"),
                }
                self.rows[motor_id] = vars_for_row

            ttk.Label(right_panel, text=str(motor_id)).grid(row=row_index, column=0, padx=4, pady=2, sticky="w")
            ttk.Label(right_panel, text=EXPECTED_MODULES[motor_id]).grid(
                row=row_index,
                column=1,
                padx=4,
                pady=2,
                sticky="w",
            )
            ttk.Label(right_panel, textvariable=vars_for_row["state"]).grid(row=row_index, column=2, padx=4, pady=2, sticky="w")
            ttk.Label(right_panel, textvariable=vars_for_row["mode"]).grid(row=row_index, column=3, padx=4, pady=2, sticky="w")
            ttk.Label(right_panel, textvariable=vars_for_row["fault"]).grid(row=row_index, column=4, padx=4, pady=2, sticky="w")
            ttk.Label(right_panel, textvariable=vars_for_row["position"]).grid(row=row_index, column=5, padx=4, pady=2, sticky="w")
            ttk.Label(right_panel, textvariable=vars_for_row["velocity"]).grid(row=row_index, column=6, padx=4, pady=2, sticky="w")
            ttk.Label(right_panel, textvariable=vars_for_row["torque"]).grid(row=row_index, column=7, padx=4, pady=2, sticky="w")

        log_frame = ttk.LabelFrame(left_panel, text="Log", padding=8)
        log_frame.pack(fill="both", expand=True, pady=(10, 0))

        self.log_text = tk.Text(log_frame, height=12, wrap="word")
        self.log_text.pack(fill="both", expand=True)
        self.log_text.configure(state="disabled")

        ttk.Label(outer, textvariable=self.status_var, padding=(0, 8, 0, 0)).pack(anchor="w")

    def _bind_keyboard_controls(self):
        for key in ["w", "W", "s", "S", "a", "A", "d", "D", "j", "J", "l", "L"]:
            self.root.bind(f"<KeyPress-{key}>", lambda event, k=key: self._on_key_press(k))
            self.root.bind(f"<KeyRelease-{key}>", lambda event, k=key: self._on_key_release(k))

    def _on_key_press(self, key: str):
        self.keyboard_pressed.add(key.lower())

    def _on_key_release(self, key: str):
        self.keyboard_pressed.discard(key.lower())

    def _poll_keyboard_controls(self):
        drive_action = None
        if "w" in self.keyboard_pressed:
            drive_action = ("north", "North", 1.0)
        elif "s" in self.keyboard_pressed:
            drive_action = ("south", "South", -1.0)

        if drive_action is not None:
            action_id, label, direction = drive_action
            if self.keyboard_drive_action_active != action_id:
                self.keyboard_drive_action_active = action_id
                self.toggle_latched_direction(label, direction, 0.0)
        else:
            self.keyboard_drive_action_active = None

        orbit_action = None
        if "j" in self.keyboard_pressed:
            orbit_action = ("orbit_east", "Orbit East", 1.0)
        elif "l" in self.keyboard_pressed:
            orbit_action = ("orbit_west", "Orbit West", -1.0)

        if orbit_action is not None:
            action_id, label, direction = orbit_action
            if self.keyboard_orbit_action_active != action_id:
                self.keyboard_orbit_action_active = action_id
                self._on_orbit_button_press(label, direction)
        else:
            if self.keyboard_orbit_action_active == "orbit_east":
                self._on_orbit_button_release("Orbit East")
            elif self.keyboard_orbit_action_active == "orbit_west":
                self._on_orbit_button_release("Orbit West")
            self.keyboard_orbit_action_active = None

        steer_action = None
        if "a" in self.keyboard_pressed:
            steer_action = ("west", "West", -1.0)
        elif "d" in self.keyboard_pressed:
            steer_action = ("east", "East", 1.0)

        if steer_action is not None:
            action_id, label, direction = steer_action
            if self.keyboard_steer_action_active != action_id:
                self.keyboard_steer_action_active = action_id
                self._on_steer_button_press(label, direction)
        else:
            if self.keyboard_steer_action_active == "west":
                self._on_steer_button_release("West")
            elif self.keyboard_steer_action_active == "east":
                self._on_steer_button_release("East")
            self.keyboard_steer_action_active = None

        self.root.after(self.keyboard_poll_ms, self._poll_keyboard_controls)

    def _append_log(self, message: str):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{time.strftime('%H:%M:%S')}  {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _refresh_direction_buttons(self):
        for label, button in self.direction_buttons.items():
            is_active = (
                label == self.current_drive_button_label
                or label == self.current_orbit_button_label
                or label == self.current_steer_button_label
            )
            if is_active:
                button.state(["pressed"])
            else:
                button.state(["!pressed"])

    def _get_settings(self) -> dict[str, float | str]:
        return {
            "port": self.port_var.get().strip(),
            "timeout": float(self.timeout_var.get().strip()),
            "drive_rps": abs(float(self.drive_rps_var.get().strip())),
            "watchdog": max(0.25, float(self.watchdog_var.get().strip())),
            "max_torque": abs(float(self.drive_torque_var.get().strip())),
            "steer_angle_deg": abs(float(self.steer_angle_var.get().strip())),
            "steer_torque": abs(float(self.steer_torque_var.get().strip())),
            "steer_vel_limit": abs(float(self.steer_vel_limit_var.get().strip())),
        }

    def _set_row_from_query(self, motor_id: int, result):
        row = self.rows[motor_id]
        row["state"].set("OK" if result.ok else "ERR")
        if not result.ok:
            row["mode"].set("-")
            row["fault"].set(result.error or "-")
            row["position"].set("-")
            row["velocity"].set("-")
            row["torque"].set("-")
            return

        row["mode"].set(fmt(read_value(result, "MODE"), 0))
        row["fault"].set(fmt(read_value(result, "FAULT"), 0))
        row["position"].set(fmt(read_value(result, "POSITION")))
        row["velocity"].set(fmt(read_value(result, "VELOCITY")))
        row["torque"].set(fmt(read_value(result, "TORQUE")))

    def _periodic_query_all(self):
        self.queue_query_all(silent=True)
        self.root.after(self.status_refresh_ms, self._periodic_query_all)


    def _on_steer_button_press(self, label: str, direction: float):
        self.current_steer_button_label = label
        self.steer_button_direction = direction
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return
        self.current_steer_angle_deg = direction * settings["steer_angle_deg"]
        self.current_steer_direction = direction
        self._refresh_direction_buttons()
        self.control_debug_var.set(
            f"buttons drive={self.current_drive_button_label} "
            f"steer={self.current_steer_button_label} angle={self.current_steer_angle_deg:.1f}"
        )
        steer_label = "right" if direction > 0.0 else "left"
        self.task_queue.put(
            ("start_steering", steer_label, self.current_steer_angle_deg, settings)
        )

    def _on_steer_button_release(self, label: str):
        if self.current_steer_button_label != label:
            return

        self.steer_button_direction = 0.0
        self.current_steer_button_label = "Center"
        self.current_steer_angle_deg = 0.0
        self.current_steer_direction = 0.0
        self._refresh_direction_buttons()
        self.control_debug_var.set(
            f"buttons drive={self.current_drive_button_label} steer=Center angle=0.0"
        )
        self.center_steering()

    def _apply_active_state(
        self,
        label: str,
        drive_direction: float,
        steer_direction: float,
        *,
        update_drive: bool = True,
        update_steer: bool = True,
    ):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        next_drive_direction = drive_direction if update_drive else self.current_drive_direction
        next_steer_direction = steer_direction if update_steer else self.current_steer_direction
        next_orbit_direction = self.current_orbit_direction

        if (
            label == self.current_joystick_label
            and next_drive_direction == self.current_drive_direction
            and next_steer_direction == self.current_steer_direction
            and next_orbit_direction == self.current_orbit_direction
        ):
            return

        self.current_joystick_label = label
        self.current_drive_direction = next_drive_direction
        self.current_steer_direction = next_steer_direction
        self.control_state_var.set(label)

        if update_drive:
            if next_orbit_direction != 0.0:
                pass
            elif next_drive_direction == 0.0:
                self.task_queue.put(("stop_motion", settings))
            else:
                drive_label = "forward" if next_drive_direction > 0.0 else "reverse"
                self.task_queue.put(("start_motion", drive_label, next_drive_direction, settings))

        if update_steer:
            if next_steer_direction == 0.0:
                self.task_queue.put(("center_steering", settings))
            else:
                steer_label = "right" if next_steer_direction > 0.0 else "left"
                steer_angle_deg = next_steer_direction * STEER_DIRECTION_SIGN * settings["steer_angle_deg"]
                self.task_queue.put(("start_steering", steer_label, steer_angle_deg, settings))

        self.status_var.set(f"Joystick {label}")

    def _apply_joystick_state(self, label: str, drive_direction: float, steer_direction: float):
        self.current_drive_button_label = "Center"
        self.current_steer_button_label = "Center"
        self._refresh_direction_buttons()
        self._apply_active_state(label, drive_direction, steer_direction)

    def _compose_button_label(self, drive_label: str, steer_label: str) -> str:
        drive_like_label = drive_label
        if self.current_orbit_button_label != "Center":
            drive_like_label = self.current_orbit_button_label
        if drive_like_label != "Center" and steer_label != "Center":
            return f"{drive_like_label}+{steer_label}"
        if drive_like_label != "Center":
            return drive_like_label
        if steer_label != "Center":
            return steer_label
        return "Center"

    def toggle_latched_direction(self, label: str, drive_direction: float, steer_direction: float):
        self.current_orbit_button_label = "Center"
        self.current_orbit_direction = 0.0
        if self.current_drive_button_label == label:
            self.current_drive_button_label = "Center"
            drive_direction = 0.0
        else:
            self.current_drive_button_label = label
        if steer_direction == 0.0 and self.current_steer_direction == 0.0:
            self.current_steer_button_label = "Center"
        self._refresh_direction_buttons()
        self.control_debug_var.set(
            f"buttons drive={self.current_drive_button_label} steer={self.current_steer_button_label}"
        )
        self._apply_active_state(
            self._compose_button_label(self.current_drive_button_label, self.current_steer_button_label),
            drive_direction,
            self.current_steer_direction,
            update_drive=True,
            update_steer=False,
        )

    def _on_orbit_button_press(self, label: str, orbit_direction: float):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        self.current_drive_button_label = "Center"
        self.current_orbit_button_label = label
        self.current_orbit_direction = orbit_direction
        self.current_drive_direction = 0.0
        self.control_state_var.set(label)
        self.control_debug_var.set(f"orbit direction={orbit_direction:+.1f}")
        self._refresh_direction_buttons()
        self.task_queue.put(("start_orbit", label, orbit_direction, settings))

    def _on_orbit_button_release(self, label: str):
        if self.current_orbit_button_label != label:
            return

        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        self.current_orbit_button_label = "Center"
        self.current_orbit_direction = 0.0
        self.current_drive_direction = 0.0
        self.control_state_var.set("Center")
        self.control_debug_var.set("orbit centered")
        self._refresh_direction_buttons()
        self.task_queue.put(("stop_motion", settings))

    def reset_direction_toggle(self):
        self.current_drive_button_label = "Center"
        self.current_steer_button_label = "Center"
        self.steer_button_direction = 0.0
        self.current_steer_angle_deg = 0.0
        self._refresh_direction_buttons()
        self.control_debug_var.set("keyboard/buttons centered")
        self._apply_active_state("Center", 0.0, 0.0)

    def start_steering(self, direction: float, label: str):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        steer_angle_deg = direction * STEER_DIRECTION_SIGN * settings["steer_angle_deg"]
        self.status_var.set(f"Steering {label}")
        self.task_queue.put(("start_steering", label, steer_angle_deg, settings))

    def center_steering(self):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        self.status_var.set("Centering steering")
        self.task_queue.put(("center_steering", settings))

    def stop_motion(self):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        self.current_drive_direction = 0.0
        self.status_var.set("Stopping motion")
        self.task_queue.put(("stop_motion", settings))

    def queue_stop_all(self):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        self.status_var.set("Queued stop all")
        self.task_queue.put(("stop", settings))

    def queue_query_drives(self):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        self.status_var.set("Queued drive query")
        self.task_queue.put(("query", settings))

    def queue_query_all(self, silent: bool = False):
        if self.status_query_pending:
            return

        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        self.status_query_pending = True
        if not silent:
            self.status_var.set("Queued full motor query")
        self.task_queue.put(("query_all", settings))

    def capture_steer_zero(self):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        self.task_queue.put(("capture_steer_zero", settings))

    def align_steer_zero(self):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        self.task_queue.put(("align_steer_zero", settings))

    def _worker_loop(self):
        active_motion = None
        active_steering = None
        while True:
            try:
                task = self.task_queue.get(timeout=self.motion_refresh_s)
            except queue.Empty:
                task = None

            try:
                if task is None:
                    if active_motion is not None:
                        label, direction, settings = active_motion
                        if label == "orbit":
                            port = asyncio.run(
                                command_orbit_motors_async(
                                    port_arg=settings["port"],
                                    timeout_s=settings["timeout"],
                                    drive_rps=settings["drive_rps"],
                                    max_torque=settings["max_torque"],
                                    orbit_direction=direction,
                                    watchdog_timeout=settings["watchdog"],
                                )
                            )
                        else:
                            port = asyncio.run(
                                command_drive_motors_async(
                                    port_arg=settings["port"],
                                    timeout_s=settings["timeout"],
                                    drive_rps=settings["drive_rps"],
                                    max_torque=settings["max_torque"],
                                    direction=direction,
                                    watchdog_timeout=settings["watchdog"],
                                )
                            )
                        self.result_queue.put(("motion_active", label, port, settings))
                    if active_steering is not None:
                        label, steer_angle_deg, settings = active_steering
                        port = asyncio.run(
                            command_steer_motors_async(
                                port_arg=settings["port"],
                                timeout_s=settings["timeout"],
                                steer_angle_deg=steer_angle_deg,
                                max_torque=settings["steer_torque"],
                                velocity_limit=settings["steer_vel_limit"],
                                watchdog_timeout=settings["watchdog"],
                                center_offsets=self.steer_center_offsets,
                            )
                        )
                        self.result_queue.put(("steering_active", label, port, settings))
                    continue

                if task[0] == "query":
                    settings = task[1]
                    port, results = asyncio.run(
                        query_drive_motors_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                        )
                    )
                    self.result_queue.put(("query_done", port, results))
                elif task[0] == "query_all":
                    settings = task[1]
                    port, results = asyncio.run(
                        query_all_motors_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                        )
                    )
                    self.result_queue.put(("query_all_done", port, results))
                elif task[0] == "capture_steer_zero":
                    settings = task[1]
                    port, results = asyncio.run(
                        query_steer_motors_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                        )
                    )
                    self.result_queue.put(("steer_zero_captured", port, results))
                elif task[0] == "align_steer_zero":
                    settings = task[1]
                    port, statuses = asyncio.run(
                        zero_steer_outputs_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                        )
                    )
                    self.result_queue.put(("steer_zero_aligned", port, statuses))
                elif task[0] == "stop":
                    settings = task[1]
                    port, statuses = asyncio.run(
                        stop_all_motors_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                        )
                    )
                    self.result_queue.put(("stop_done", port, statuses))
                    active_motion = None
                    active_steering = None
                elif task[0] == "start_motion":
                    _kind, label, direction, settings = task
                    active_motion = (label, direction, settings)
                    port = asyncio.run(
                        command_drive_motors_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                            drive_rps=settings["drive_rps"],
                            max_torque=settings["max_torque"],
                            direction=direction,
                            watchdog_timeout=settings["watchdog"],
                        )
                    )
                    self.result_queue.put(("motion_started", label, port, settings))
                elif task[0] == "start_orbit":
                    _kind, label, orbit_direction, settings = task
                    active_motion = ("orbit", orbit_direction, settings)
                    port = asyncio.run(
                        command_orbit_motors_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                            drive_rps=settings["drive_rps"],
                            max_torque=settings["max_torque"],
                            orbit_direction=orbit_direction,
                            watchdog_timeout=settings["watchdog"],
                        )
                    )
                    self.result_queue.put(("orbit_started", label, port, settings))
                elif task[0] == "start_steering":
                    _kind, label, steer_angle_deg, settings = task
                    active_steering = (label, steer_angle_deg, settings)
                    port = asyncio.run(
                        command_steer_motors_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                            steer_angle_deg=steer_angle_deg,
                            max_torque=settings["steer_torque"],
                            velocity_limit=settings["steer_vel_limit"],
                            watchdog_timeout=settings["watchdog"],
                            center_offsets=self.steer_center_offsets,
                        )
                    )
                    self.result_queue.put(("steering_started", label, port, settings))
                elif task[0] == "center_steering":
                    settings = task[1]
                    active_steering = ("center", 0.0, settings)
                    port = asyncio.run(
                        command_steer_motors_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                            steer_angle_deg=0.0,
                            max_torque=settings["steer_torque"],
                            velocity_limit=settings["steer_vel_limit"],
                            watchdog_timeout=settings["watchdog"],
                            center_offsets=self.steer_center_offsets,
                        )
                    )
                    self.result_queue.put(("steering_centered", port, settings))
                elif task[0] == "stop_motion":
                    settings = task[1]
                    port, statuses = asyncio.run(
                        stop_drive_motors_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                        )
                    )
                    active_motion = None
                    self.result_queue.put(("motion_stopped", port, statuses))
            except Exception as exc:
                self.result_queue.put(("error", repr(exc)))

    def _poll_results(self):
        while True:
            try:
                item = self.result_queue.get_nowait()
            except queue.Empty:
                break

            kind = item[0]
            if kind == "error":
                self.status_query_pending = False
                self.status_var.set(f"Error: {item[1]}")
                self._append_log(f"ERROR {item[1]}")
            elif kind == "query_done":
                self.status_query_pending = False
                _kind, port, results = item
                for result in results:
                    self._set_row_from_query(result.motor_id, result)
                self.status_var.set(f"Queried drives on {port or 'default transport'}")
                self._append_log("Drive query completed")
            elif kind == "query_all_done":
                self.status_query_pending = False
                _kind, port, results = item
                for result in results:
                    self._set_row_from_query(result.motor_id, result)
                if any(not result.ok for result in results):
                    self._append_log(
                        "Query all warnings: "
                        + ", ".join(
                            f"{result.motor_id}={result.error}"
                            for result in results
                            if not result.ok
                        )
                    )
                self.status_var.set(f"Queried all motors on {port or 'default transport'}")
            elif kind == "steer_zero_captured":
                _kind, port, results = item
                captured = []
                for result in results:
                    if not result.ok:
                        continue
                    position = read_value(result, "ABS_POSITION")
                    if position is None:
                        position = read_value(result, "POSITION")
                    if position is None:
                        continue
                    self.steer_center_offsets[result.motor_id] = float(position)
                    captured.append(f"{result.motor_id}={float(position):.3f}")
                    self._set_row_from_query(result.motor_id, result)
                if captured:
                    self._append_log(
                        "Steer zero captured on "
                        f"{port or 'default transport'}: " + ", ".join(captured)
                    )
                else:
                    self._append_log("Steer zero capture returned no valid positions")
            elif kind == "steer_zero_aligned":
                _kind, port, statuses = item
                captured = []
                for motor_id, status in statuses.items():
                    if "error" in status:
                        captured.append(f"{motor_id}=ERR:{status['error']}")
                        continue
                    position = status.get("abs_position")
                    if position is None:
                        position = status.get("position")
                    if position is None:
                        captured.append(f"{motor_id}=no_position")
                        continue
                    self.steer_center_offsets[motor_id] = float(position)
                    captured.append(f"{motor_id}={float(position):.3f}")
                self._append_log(
                    "Steer output aligned on "
                    f"{port or 'default transport'}: " + ", ".join(captured)
                )
            elif kind == "stop_done":
                _kind, port, statuses = item
                self.current_drive_direction = 0.0
                self.current_orbit_direction = 0.0
                self.current_steer_direction = 0.0
                self.current_steer_angle_deg = 0.0
                self.steer_button_direction = 0.0
                self.current_joystick_label = "Center"
                self.current_drive_button_label = "Center"
                self.current_orbit_button_label = "Center"
                self.current_steer_button_label = "Center"
                self.control_state_var.set("Center")
                self.control_debug_var.set("keyboard/buttons centered")
                self._refresh_direction_buttons()
                self.status_var.set(f"Stop all sent on {port or 'default transport'}")
                self._append_log(f"Stop statuses: {statuses}")
                self.align_steer_zero()
                self.queue_query_all(silent=True)
            elif kind == "motion_started":
                _kind, label, port, settings = item
                self.status_var.set(f"Driving {label} on {port or 'default transport'}")
                self._append_log(
                    f"{label.title()} motion started: speed={settings['drive_rps']:.2f} rev/s, "
                    f"watchdog={settings['watchdog']:.2f} s, torque={settings['max_torque']:.2f}"
                )
            elif kind == "motion_active":
                _kind, label, port, _settings = item
                if label == "orbit":
                    orbit_label = "orbiting east" if self.current_orbit_direction > 0.0 else "orbiting west"
                    self.status_var.set(f"{orbit_label.title()} on {port or 'default transport'}")
                else:
                    self.status_var.set(f"Driving {label} on {port or 'default transport'}")
            elif kind == "orbit_started":
                _kind, label, port, settings = item
                self.status_var.set(f"{label} on {port or 'default transport'}")
                self._append_log(
                    f"{label}: speed={settings['drive_rps']:.2f} rev/s, "
                    f"watchdog={settings['watchdog']:.2f} s, torque={settings['max_torque']:.2f}"
                )
            elif kind == "steering_started":
                _kind, label, port, settings = item
                self.status_var.set(f"Steered {label} on {port or 'default transport'}")
                self._append_log(
                    f"Steer {label}: angle={settings['steer_angle_deg']:.1f} deg, "
                    f"vel_limit={settings['steer_vel_limit']:.2f}, torque={settings['steer_torque']:.2f}"
                )
            elif kind == "steering_active":
                _kind, label, port, _settings = item
                self.status_var.set(f"Steering held {label} on {port or 'default transport'}")
            elif kind == "steering_centered":
                _kind, port, _settings = item
                self.current_steer_angle_deg = 0.0
                self.status_var.set(f"Steering centered on {port or 'default transport'}")
                self._append_log("Steering returned to center")
                self.queue_query_all(silent=True)
            elif kind == "motion_stopped":
                _kind, port, statuses = item
                self.current_drive_direction = 0.0
                self.current_drive_button_label = "Center"
                self.current_orbit_direction = 0.0
                self.current_orbit_button_label = "Center"
                self.control_debug_var.set("keyboard/buttons centered")
                self._refresh_direction_buttons()
                self.status_var.set(f"Motion stopped on {port or 'default transport'}")
                self._append_log(f"Motion stop statuses: {statuses}")

        self.root.after(100, self._poll_results)


def main():
    root = tk.Tk()
    app = DriveBumpGuiApp(root)
    try:
        root.mainloop()
    finally:
        try:
            settings = app._get_settings()
            asyncio.run(
                stop_all_motors_async(
                    port_arg=str(settings["port"]),
                    timeout_s=float(settings["timeout"]),
                )
            )
        except Exception:
            pass


def ros_main(args=None):
    rclpy.init(args=args)
    node = UsamaControlRosNode()
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


def cli_main():
    parser = argparse.ArgumentParser(
        description="CMR direct moteus drive test GUI or ROS joystick drive node."
    )
    parser.add_argument(
        "--mode",
        choices=["gui", "ros"],
        default="gui",
        help="Run the Tkinter GUI or the ROS topic subscriber node.",
    )
    args, ros_args = parser.parse_known_args()

    if args.mode == "ros":
        ros_main(ros_args)
    else:
        main()


if __name__ == "__main__":
    cli_main()
