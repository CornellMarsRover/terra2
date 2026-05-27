#!/usr/bin/env python3

from __future__ import annotations

import asyncio
import math
import threading
import time
from dataclasses import dataclass
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
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


DRIVE_IDS = [1, 2, 3, 4]
STEER_IDS = [5, 7, 6, 8]
STEER_DEGREES_TO_POSITION = 50.0 / 360.0

FORWARD_SIGN = {
    1: 1.0,
    2: 1.0,
    3: -1.0,
    4: -1.0,
}

STEER_SIGN = {
    5: 1.0,
    7: 1.0,
    6: 1.0,
    8: 1.0,
}

STEER_CENTER_OFFSETS = {
    5: 0.0,
    7: 0.0,
    6: 0.0,
    8: 0.0,
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
TRIGGER_PRESSED_THRESHOLD = 32
HALF_SPEED_MULTIPLIER = 0.5
DOUBLE_SPEED_MULTIPLIER = 2.0
TRIPLE_SPEED_MULTIPLIER = 3.0

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


async def query_steer_motors_async(port_arg: str, timeout_s: float):
    port, controllers = make_transport_and_controllers(port_arg, STEER_IDS)
    results = []
    for motor_id in STEER_IDS:
        label, ctrl = controllers[motor_id]
        results.append(await query_one(ctrl, motor_id, label, timeout_s))
    return port, results


async def stop_all_motors_async(port_arg: str, timeout_s: float):
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


def calculate_swerve_targets(
    vx: float,
    vy: float,
    omega: float,
    drive_rps: float,
    center_offsets: dict[int, float],
):
    radius = math.sqrt(ROVER_LENGTH**2 + ROVER_WIDTH**2)

    vx = round(vx, 3)
    vy = round(vy, 3)
    omega = round(omega, 3)

    a = vy - omega * (ROVER_LENGTH / radius)
    b = vy + omega * (ROVER_LENGTH / radius)
    c = vx - omega * (ROVER_WIDTH / radius)
    d = vx + omega * (ROVER_WIDTH / radius)

    ws1 = math.sqrt(b**2 + c**2)
    ws2 = math.sqrt(b**2 + d**2)
    ws3 = math.sqrt(a**2 + d**2)
    ws4 = math.sqrt(a**2 + c**2)

    max_speed = max(ws1, ws2, ws3, ws4, 1.0)
    ws1 /= max_speed
    ws2 /= max_speed
    ws3 /= max_speed
    ws4 /= max_speed

    wa1 = round(math.atan2(b, c) * 180.0 / math.pi, 3) * STEER_DEGREES_TO_POSITION
    wa2 = round(math.atan2(b, d) * 180.0 / math.pi, 3) * STEER_DEGREES_TO_POSITION
    wa3 = round(math.atan2(a, d) * 180.0 / math.pi, 3) * STEER_DEGREES_TO_POSITION
    wa4 = round(math.atan2(a, c) * 180.0 / math.pi, 3) * STEER_DEGREES_TO_POSITION

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
    def __init__(self) -> None:
        super().__init__("usama_control_testing_node")

        self.declare_parameter("config_path", "")
        config = self._load_node_config()

        self.declare_parameter("can_port", "/dev/ttyACM0")
        self.declare_parameter("timeout_s", 0.30)
        self.declare_parameter("drive_rps", 4.0)
        self.declare_parameter("half_speed_multiplier", HALF_SPEED_MULTIPLIER)
        self.declare_parameter("double_speed_multiplier", DOUBLE_SPEED_MULTIPLIER)
        self.declare_parameter("triple_speed_multiplier", TRIPLE_SPEED_MULTIPLIER)
        self.declare_parameter("trigger_pressed_threshold", TRIGGER_PRESSED_THRESHOLD)
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
        self.manual_override_priority = bool(
            self._setting("manual_override_priority", config)
        )
        self.autonomy_priority = bool(self._setting("autonomy_priority", config))
        self.capture_steer_zero_on_start = bool(
            self._setting("capture_steer_zero_on_start", config)
        )

        self._manual = ManualCommandState()
        self._autonomy = AutonomyCommandState()
        self._estop_latched = False
        self._last_source = "idle"
        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._steer_center_offsets = dict(STEER_CENTER_OFFSETS)
        self._last_manual_drive_axis_sign = 1.0

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
        self.create_subscription(
            Twist,
            "/cmd_vel_drives",
            self._autonomy_cmd_vel_cb,
            10,
        )

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "usama_control_testing_node started. Inputs: /drives_controller/cmd_vel, "
            "/drives_controller/cmd_buttons, /cmd_vel_drives"
        )
        self.get_logger().info(
            f"Using direct moteus command behavior on {self.port}: "
            f"drive_rps={self.drive_rps:.2f}, drive_torque={self.drive_max_torque:.2f}, "
            f"steer_torque={self.steer_max_torque:.2f}, "
            f"half={self.half_speed_multiplier:.2f}x, "
            f"double={self.double_speed_multiplier:.2f}x, "
            f"triple={self.triple_speed_multiplier:.2f}x"
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
        vx = self._apply_deadzone(
            -msg.twist.linear.y / STICK_RANGE,
            self.controller_deadzone,
        )
        vy = self._apply_deadzone(
            msg.twist.linear.x / STICK_RANGE,
            self.controller_deadzone,
        )
        omega = self._apply_deadzone(
            msg.twist.angular.x / STICK_RANGE,
            self.controller_deadzone,
        )

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
                f"/drives_controller/cmd_buttons expected 2 or 8 entries, got "
                f"{len(msg.button_array)}",
                throttle_duration_sec=2.0,
            )
            return

        l1 = decoded["l1"]
        r1 = decoded["r1"]
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

            speed_rps = self._manual_speed_rps(l1=l1, r1=r1, l2=l2, r2=r2)

            self._manual.speed_rps = speed_rps
            self._manual.updated_at = time.time()

        self.get_logger().info(
            f"controller buttons L1={l1} R1={r1} L2={l2} R2={r2} "
            f"speed_rps={speed_rps:+.3f}",
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
            f"vy={self._clamp(msg.linear.y):+.3f} "
            f"omega={self._clamp(msg.angular.z):+.3f}",
            throttle_duration_sec=1.0,
        )

    def _worker_loop(self) -> None:
        self._initialize_hardware_state()
        refresh_period = 1.0 / max(self.refresh_rate_hz, 1.0)

        while not self._shutdown.is_set():
            try:
                command = self._select_command()
                self.get_logger().info(
                    f"selected drive command: {command}",
                    throttle_duration_sec=1.0,
                )
                if command["mode"] in {"idle", "estop"}:
                    if self._last_source != command["source"]:
                        self.get_logger().info(
                            f"Drive source switched to {command['source']}"
                        )
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
                            f"Drive source switched to {command['source']}; "
                            f"targets={targets}"
                        )
                    self._last_source = str(command["source"])
            except Exception as exc:
                self.get_logger().error(
                    f"Drive worker error: {exc!r}",
                    throttle_duration_sec=1.0,
                )
            finally:
                time.sleep(refresh_period)

        try:
            asyncio.run(stop_all_motors_async(self.port, self.timeout_s))
        except Exception as exc:
            self.get_logger().error(f"Drive shutdown stop failed: {exc!r}")

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
            or any(
                abs(value) > self.controller_deadzone
                for value in (manual.vx, manual.vy, manual.omega)
            )
        )
        autonomy_active = (now - autonomy.updated_at) <= self.command_timeout_s and any(
            abs(value) > COMMAND_EPSILON
            for value in (autonomy.vx, autonomy.vy, autonomy.omega)
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
        speed_rps = manual.speed_rps * drive_axis_sign
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
        except (RuntimeError, KeyboardInterrupt):
            pass
        super().destroy_node()

    @staticmethod
    def _decode_controller_buttons(buttons: list[int]) -> dict[str, int | str] | None:
        if len(buttons) >= 8:
            return {
                "format": "expanded",
                "l1": int(buttons[BTN_L1]),
                "r1": int(buttons[BTN_R1]),
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
                "r1": 0,
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
            return int(
                round(
                    (
                        (trigger_val - LEGACY_L2_MIN)
                        / (LEGACY_L2 - LEGACY_L2_MIN)
                    )
                    * TRIGGER_MAX
                )
            )
        return 0

    def _trigger_pressed(self, trigger_byte: int) -> bool:
        value = int(trigger_byte)
        return value >= max(TRIGGER_DEADBAND, self.trigger_pressed_threshold)

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

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        return 0.0 if abs(value) < deadzone else value

    @staticmethod
    def _clamp(value: float, limit: float = 1.0) -> float:
        return max(-limit, min(limit, value))


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


if __name__ == "__main__":
    ros_main()
