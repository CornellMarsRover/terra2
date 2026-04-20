#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import moteus
import rclpy
from cmr_msgs.msg import ControllerReading
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


DRIVE_IDS = {
    "front_left": 1,
    "back_left": 2,
    "front_right": 3,
    "back_right": 4,
}

STEER_IDS = {
    "front_left": 5,
    "back_left": 6,
    "front_right": 7,
    "back_right": 8,
}

MOTOR_LABELS = {
    1: "front_left_drive",
    2: "back_left_drive",
    3: "front_right_drive",
    4: "back_right_drive",
    5: "front_left_steer",
    6: "back_left_steer",
    7: "front_right_steer",
    8: "back_right_steer",
}

BTN_L1 = 0
BTN_R1 = 1
BTN_L2 = 2
BTN_R2 = 3
BTN_SQUARE = 4
BTN_CROSS = 5
BTN_CIRCLE = 6
BTN_TRIANGLE = 7

STICK_RANGE = 2.5
TRIGGER_DEADBAND = 5
TRIGGER_MAX = 255

ROVER_LENGTH = 39.0
ROVER_WIDTH = 39.0
SWERVE_RATIO = 50.0

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_LOG_ROOT = REPO_ROOT / "logs" / "driving_sessions"


@dataclass
class ManualState:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    speed_rps: float = 0.0
    updated_at: float = 0.0


@dataclass
class AutonomyState:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    updated_at: float = 0.0


class ZennyDrivesNode(Node):
    """Unified rover drive executor.

    This node is the single place that turns ROS drive intents into Moteus
    commands for the rover drive + steer motors.

    Inputs:
    - /drives_controller/cmd_vel      (manual controller stick geometry)
    - /drives_controller/cmd_buttons  (manual triggers + estop)
    - /cmd_vel_drives                 (standardized autonomy/teleop twist)

    Architectural intent:
    - `cmr_controller_remote/connect.py` remains the UDP -> ROS ingress.
    - This node becomes the only ROS -> rover drive executor.
    - Autonomy should publish to `/cmd_vel_drives` instead of faking controller
      packets or publishing raw controller topics.
    """

    def __init__(self) -> None:
        super().__init__("zenny_drives_node")

        self.declare_parameter("can_port", "/dev/ttyACM0")
        self.declare_parameter("motor_max_speed", 40.0)
        self.declare_parameter("drive_max_torque", 8.0)
        self.declare_parameter("drive_acceleration_limit", 10.0)
        self.declare_parameter("steer_max_torque", 10.0)
        self.declare_parameter("steer_velocity_limit", 60.0)
        self.declare_parameter("steer_acceleration_limit", 40.0)
        self.declare_parameter("controller_deadzone", 0.1)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("refresh_rate_hz", 10.0)
        self.declare_parameter("autonomy_priority", True)
        self.declare_parameter("state_poll_hz", 2.0)
        self.declare_parameter("session_log_root", str(DEFAULT_LOG_ROOT))

        self.can_port = str(self.get_parameter("can_port").value)
        self.motor_max_speed = float(self.get_parameter("motor_max_speed").value)
        self.drive_max_torque = float(self.get_parameter("drive_max_torque").value)
        self.drive_acceleration_limit = float(self.get_parameter("drive_acceleration_limit").value)
        self.steer_max_torque = float(self.get_parameter("steer_max_torque").value)
        self.steer_velocity_limit = float(self.get_parameter("steer_velocity_limit").value)
        self.steer_acceleration_limit = float(self.get_parameter("steer_acceleration_limit").value)
        self.controller_deadzone = float(self.get_parameter("controller_deadzone").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.refresh_rate_hz = float(self.get_parameter("refresh_rate_hz").value)
        self.autonomy_priority = bool(self.get_parameter("autonomy_priority").value)
        self.state_poll_hz = float(self.get_parameter("state_poll_hz").value)
        self.session_log_root = Path(str(self.get_parameter("session_log_root").value)).expanduser()

        self._manual = ManualState()
        self._autonomy = AutonomyState()
        self._estop_latched = False
        self._last_source = "idle"
        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._last_state_query_at = 0.0

        self._session_started_at = time.time()
        self._log_lock = threading.Lock()
        self._log_sequence = 0
        self._session_dir = self._make_session_dir()
        self._log_files = self._open_log_files()
        self._write_session_metadata()

        self.create_subscription(TwistStamped, "/drives_controller/cmd_vel", self._controller_cmd_vel_cb, 10)
        self.create_subscription(
            ControllerReading, "/drives_controller/cmd_buttons", self._controller_buttons_cb, 10
        )
        self.create_subscription(Twist, "/cmd_vel_drives", self._autonomy_cmd_vel_cb, 10)

        self._transport = None
        self._controllers = {}
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "zenny_drives_node started. Inputs: /drives_controller/cmd_vel, "
            "/drives_controller/cmd_buttons, /cmd_vel_drives"
        )
        self.get_logger().info(f"Drive session logs: {self._session_dir}")

    def _controller_cmd_vel_cb(self, msg: TwistStamped) -> None:
        vx = self._apply_deadzone(-msg.twist.linear.y / STICK_RANGE, self.controller_deadzone)
        vy = self._apply_deadzone(msg.twist.linear.x / STICK_RANGE, self.controller_deadzone)
        omega = self._apply_deadzone(msg.twist.angular.x / STICK_RANGE, self.controller_deadzone)

        manual_update = {
            "vx": self._clamp(vx),
            "vy": self._clamp(vy),
            "omega": self._clamp(omega),
            "stamp": self._ros_time_to_dict(msg.header.stamp),
        }

        with self._lock:
            self._manual.vx = manual_update["vx"]
            self._manual.vy = manual_update["vy"]
            self._manual.omega = manual_update["omega"]
            self._manual.updated_at = time.time()

        self._log_event(
            "controller_cmd_vel",
            {
                "raw": {
                    "linear_x": float(msg.twist.linear.x),
                    "linear_y": float(msg.twist.linear.y),
                    "angular_x": float(msg.twist.angular.x),
                    "angular_y": float(msg.twist.angular.y),
                },
                "normalized": manual_update,
            },
        )

    def _controller_buttons_cb(self, msg: ControllerReading) -> None:
        buttons = list(msg.button_array)
        if len(buttons) < 8:
            self.get_logger().warn(
                f"/drives_controller/cmd_buttons expected 8 entries, got {len(buttons)}",
                throttle_duration_sec=2.0,
            )
            self._log_event(
                "controller_cmd_buttons",
                {"error": "button_array_too_short", "length": len(buttons), "values": buttons},
            )
            return

        l1 = int(buttons[BTN_L1])
        triangle = int(buttons[BTN_TRIANGLE])
        l2 = int(buttons[BTN_L2])
        r2 = int(buttons[BTN_R2])

        estop_pressed = bool(l1) and bool(triangle)
        event = {
            "buttons": buttons,
            "decoded": {"l1": l1, "triangle": triangle, "l2": l2, "r2": r2},
            "estop_pressed": estop_pressed,
        }

        with self._lock:
            if estop_pressed and not self._estop_latched:
                self._estop_latched = True
                self._manual.speed_rps = 0.0
                self._manual.updated_at = time.time()
                self.get_logger().warn("Drive estop latched from controller buttons")
                event["result"] = {"estop_latched": True, "speed_rps": 0.0}
                self._log_event("controller_cmd_buttons", event)
                return

            if self._estop_latched and not bool(l1) and not bool(triangle):
                self._estop_latched = False
                self.get_logger().info("Drive estop released")

            if self._estop_latched:
                event["result"] = {"estop_latched": True, "speed_rps": 0.0}
                self._log_event("controller_cmd_buttons", event)
                return

            forward_rps = self._trigger_to_speed_scale(r2) * self.motor_max_speed
            reverse_rps = self._trigger_to_speed_scale(l2) * self.motor_max_speed
            if forward_rps > 0.0:
                speed_rps = forward_rps
            elif reverse_rps > 0.0:
                speed_rps = -reverse_rps
            else:
                speed_rps = 0.0

            self._manual.speed_rps = speed_rps
            self._manual.updated_at = time.time()

        event["result"] = {
            "estop_latched": self._estop_latched,
            "forward_rps": forward_rps,
            "reverse_rps": reverse_rps,
            "speed_rps": speed_rps,
        }
        self._log_event("controller_cmd_buttons", event)

    def _autonomy_cmd_vel_cb(self, msg: Twist) -> None:
        autonomy_update = {
            "vx": self._clamp(msg.linear.x),
            "vy": self._clamp(msg.linear.y),
            "omega": self._clamp(msg.angular.z),
        }

        with self._lock:
            self._autonomy.vx = autonomy_update["vx"]
            self._autonomy.vy = autonomy_update["vy"]
            self._autonomy.omega = autonomy_update["omega"]
            self._autonomy.updated_at = time.time()

        self._log_event(
            "autonomy_cmd_vel",
            {
                "raw": {
                    "linear_x": float(msg.linear.x),
                    "linear_y": float(msg.linear.y),
                    "angular_z": float(msg.angular.z),
                },
                "normalized": autonomy_update,
            },
        )

    def _worker_loop(self) -> None:
        try:
            asyncio.run(self._async_initialize_hardware())
        except Exception as exc:  # pragma: no cover - hardware path
            self.get_logger().error(f"Failed to initialize drive hardware: {exc!r}")
            self._log_event("worker_errors", {"stage": "initialize_hardware", "error": repr(exc)})
            return

        refresh_period = 1.0 / max(self.refresh_rate_hz, 1.0)
        while not self._shutdown.is_set():
            try:
                command = self._select_command()
                self._log_event("selected_command", command)
                query_state = self._should_query_state()

                if command["mode"] == "estop":
                    if self._last_source != "estop":
                        self.get_logger().warn("Applying drive estop")
                    asyncio.run(self._async_stop_all())
                    self._last_source = "estop"
                elif command["mode"] == "idle":
                    if self._last_source != "idle":
                        self.get_logger().info("No recent drive command; stopping motors")
                    asyncio.run(self._async_stop_all())
                    self._last_source = "idle"
                else:
                    wheel_targets = self._command_to_wheels(command)
                    self._log_event(
                        "wheel_targets",
                        {
                            "source": command["source"],
                            "mode": command["mode"],
                            "query_state": query_state,
                            "targets": wheel_targets,
                        },
                    )
                    state_snapshot = asyncio.run(self._async_apply_wheels(wheel_targets, query_state=query_state))
                    if state_snapshot is not None:
                        self._log_event("wheel_state", state_snapshot)
                    if command["source"] != self._last_source:
                        self.get_logger().info(f"Drive command source switched to {command['source']}")
                    self._last_source = command["source"]
            except Exception as exc:  # pragma: no cover - hardware path
                self.get_logger().error(f"Drive worker error: {exc!r}")
                self._log_event("worker_errors", {"stage": "worker_loop", "error": repr(exc)})
            finally:
                time.sleep(refresh_period)

    def _select_command(self) -> dict[str, object]:
        now = time.time()
        with self._lock:
            if self._estop_latched:
                return {"mode": "estop", "source": "controller_estop", "selected_at": now}

            manual = ManualState(**self._manual.__dict__)
            autonomy = AutonomyState(**self._autonomy.__dict__)

        autonomy_active = (now - autonomy.updated_at) <= self.command_timeout_s and any(
            abs(value) > 1e-3 for value in (autonomy.vx, autonomy.vy, autonomy.omega)
        )
        manual_active = (now - manual.updated_at) <= self.command_timeout_s and (
            abs(manual.speed_rps) > 1e-3
            or any(abs(value) > self.controller_deadzone for value in (manual.vx, manual.vy, manual.omega))
        )

        if autonomy_active and self.autonomy_priority:
            return {
                "mode": "swerve",
                "source": "autonomy_cmd_vel",
                "vx": autonomy.vx,
                "vy": autonomy.vy,
                "omega": autonomy.omega,
                "speed_scale": self.motor_max_speed,
                "selected_at": now,
            }
        if manual_active:
            return {
                "mode": "swerve",
                "source": "controller_topics",
                "vx": manual.vx,
                "vy": manual.vy,
                "omega": manual.omega,
                "speed_scale": abs(manual.speed_rps),
                "direction_sign": 1.0 if manual.speed_rps >= 0.0 else -1.0,
                "selected_at": now,
            }
        if autonomy_active:
            return {
                "mode": "swerve",
                "source": "autonomy_cmd_vel",
                "vx": autonomy.vx,
                "vy": autonomy.vy,
                "omega": autonomy.omega,
                "speed_scale": self.motor_max_speed,
                "selected_at": now,
            }
        return {"mode": "idle", "source": "idle", "selected_at": now}

    def _command_to_wheels(self, command: dict[str, object]) -> dict[str, float]:
        ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4 = self._wheel_angles_and_speeds(
            float(command["vx"]),
            float(command["vy"]),
            float(command["omega"]),
            ROVER_LENGTH,
            ROVER_WIDTH,
        )

        speed_scale = float(command["speed_scale"])
        direction_sign = float(command.get("direction_sign", 1.0))

        return {
            "front_left_drive": direction_sign * -speed_scale * ws2,
            "back_left_drive": direction_sign * -speed_scale * ws3,
            "front_right_drive": direction_sign * speed_scale * ws4,
            "back_right_drive": direction_sign * speed_scale * ws1,
            "front_left_steer": wa2,
            "back_left_steer": wa3,
            "front_right_steer": wa1,
            "back_right_steer": wa4,
        }

    async def _async_initialize_hardware(self) -> None:
        self._transport = self._make_transport(self.can_port)
        all_ids = [*DRIVE_IDS.values(), *STEER_IDS.values()]
        query_resolution = self._make_query_resolution()
        self._controllers = {
            motor_id: self._make_controller(motor_id, query_resolution) for motor_id in all_ids
        }

        await self._transport.cycle([controller.make_stop() for controller in self._controllers.values()])
        await self._transport.cycle([controller.make_rezero() for controller in self._controllers.values()])
        self._log_event(
            "hardware_init",
            {
                "can_port": self.can_port,
                "motor_ids": all_ids,
                "query_resolution_enabled": query_resolution is not None,
            },
        )

    async def _async_stop_all(self) -> None:
        if not self._transport or not self._controllers:
            return
        await self._transport.cycle([controller.make_stop() for controller in self._controllers.values()])
        self._log_event("motor_stop", {"reason": self._last_source})

    async def _async_apply_wheels(
        self, wheels: dict[str, float], query_state: bool = False
    ) -> dict[str, object] | None:
        command_specs = [
            {
                "motor_id": DRIVE_IDS["front_left"],
                "name": "front_left_drive",
                "kind": "drive",
                "position": math.nan,
                "velocity": wheels["front_left_drive"],
                "accel_limit": self.drive_acceleration_limit,
                "velocity_limit": self.motor_max_speed,
                "maximum_torque": self.drive_max_torque,
                "query": query_state,
            },
            {
                "motor_id": DRIVE_IDS["back_left"],
                "name": "back_left_drive",
                "kind": "drive",
                "position": math.nan,
                "velocity": wheels["back_left_drive"],
                "accel_limit": self.drive_acceleration_limit,
                "velocity_limit": self.motor_max_speed,
                "maximum_torque": self.drive_max_torque,
                "query": query_state,
            },
            {
                "motor_id": DRIVE_IDS["front_right"],
                "name": "front_right_drive",
                "kind": "drive",
                "position": math.nan,
                "velocity": wheels["front_right_drive"],
                "accel_limit": self.drive_acceleration_limit,
                "velocity_limit": self.motor_max_speed,
                "maximum_torque": self.drive_max_torque,
                "query": query_state,
            },
            {
                "motor_id": DRIVE_IDS["back_right"],
                "name": "back_right_drive",
                "kind": "drive",
                "position": math.nan,
                "velocity": wheels["back_right_drive"],
                "accel_limit": self.drive_acceleration_limit,
                "velocity_limit": self.motor_max_speed,
                "maximum_torque": self.drive_max_torque,
                "query": query_state,
            },
            {
                "motor_id": STEER_IDS["front_left"],
                "name": "front_left_steer",
                "kind": "steer",
                "position": wheels["front_left_steer"],
                "accel_limit": self.steer_acceleration_limit,
                "velocity_limit": self.steer_velocity_limit,
                "maximum_torque": self.steer_max_torque,
                "query": query_state,
            },
            {
                "motor_id": STEER_IDS["back_left"],
                "name": "back_left_steer",
                "kind": "steer",
                "position": wheels["back_left_steer"],
                "accel_limit": self.steer_acceleration_limit,
                "velocity_limit": self.steer_velocity_limit,
                "maximum_torque": self.steer_max_torque,
                "query": query_state,
            },
            {
                "motor_id": STEER_IDS["front_right"],
                "name": "front_right_steer",
                "kind": "steer",
                "position": wheels["front_right_steer"],
                "accel_limit": self.steer_acceleration_limit,
                "velocity_limit": self.steer_velocity_limit,
                "maximum_torque": self.steer_max_torque,
                "query": query_state,
            },
            {
                "motor_id": STEER_IDS["back_right"],
                "name": "back_right_steer",
                "kind": "steer",
                "position": wheels["back_right_steer"],
                "accel_limit": self.steer_acceleration_limit,
                "velocity_limit": self.steer_velocity_limit,
                "maximum_torque": self.steer_max_torque,
                "query": query_state,
            },
        ]

        commands = []
        for spec in command_specs:
            kwargs = {
                "position": spec["position"],
                "query": spec["query"],
                "accel_limit": spec["accel_limit"],
                "velocity_limit": spec["velocity_limit"],
                "maximum_torque": spec["maximum_torque"],
            }
            if "velocity" in spec:
                kwargs["velocity"] = spec["velocity"]
            commands.append(self._controllers[spec["motor_id"]].make_position(**kwargs))
        self._log_event("motor_commands", {"commands": self._clean_command_specs(command_specs)})

        result = await self._transport.cycle(commands)
        if not query_state:
            return None
        self._last_state_query_at = time.time()
        return {
            "queried_at": self._now_iso(),
            "motors": self._format_cycle_result(command_specs, result),
        }

    def _make_controller(self, motor_id: int, query_resolution):
        if query_resolution is None:
            return moteus.Controller(id=motor_id, transport=self._transport)
        return moteus.Controller(id=motor_id, transport=self._transport, query_resolution=query_resolution)

    @staticmethod
    def _make_transport(serial_port: str):
        try:
            return moteus.Fdcanusb(serial_port)
        except TypeError:
            pass
        try:
            return moteus.Fdcanusb(port=serial_port)
        except TypeError:
            pass
        return moteus.Fdcanusb()

    @staticmethod
    def _make_query_resolution():
        query_resolution_cls = getattr(moteus, "QueryResolution", None)
        if query_resolution_cls is None:
            return None

        query_resolution = query_resolution_cls()
        if hasattr(query_resolution, "mode") and hasattr(moteus, "INT8"):
            query_resolution.mode = moteus.INT8
        for attr in ("position", "velocity", "torque", "q_current"):
            if hasattr(query_resolution, attr) and hasattr(moteus, "F32"):
                setattr(query_resolution, attr, moteus.F32)
        return query_resolution

    def _should_query_state(self) -> bool:
        if self.state_poll_hz <= 0.0:
            return False
        period = 1.0 / self.state_poll_hz
        return (time.time() - self._last_state_query_at) >= period

    def _format_cycle_result(self, command_specs: list[dict[str, object]], result) -> list[dict[str, object]]:
        motor_states = []
        for spec, state in zip(command_specs, result):
            values = getattr(state, "values", {}) or {}
            motor_states.append(
                {
                    "motor_id": spec["motor_id"],
                    "name": spec["name"],
                    "kind": spec["kind"],
                    "command": {
                        "position": self._clean_number(spec["position"]),
                        "velocity": self._clean_number(spec.get("velocity")),
                    },
                    "feedback": {
                        "position": self._clean_number(values.get(getattr(moteus.Register, "POSITION", None))),
                        "velocity": self._clean_number(values.get(getattr(moteus.Register, "VELOCITY", None))),
                        "torque": self._clean_number(values.get(getattr(moteus.Register, "TORQUE", None))),
                        "q_current": self._clean_number(values.get(getattr(moteus.Register, "Q_CURRENT", None))),
                        "mode": self._clean_number(values.get(getattr(moteus.Register, "MODE", None))),
                    },
                }
            )
        return motor_states

    def _make_session_dir(self) -> Path:
        timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        session_dir = self.session_log_root / f"{timestamp}_{self.get_name()}"
        session_dir.mkdir(parents=True, exist_ok=True)
        return session_dir

    def _open_log_files(self) -> dict[str, object]:
        names = [
            "controller_cmd_vel",
            "controller_cmd_buttons",
            "autonomy_cmd_vel",
            "selected_command",
            "wheel_targets",
            "motor_commands",
            "wheel_state",
            "motor_stop",
            "hardware_init",
            "worker_errors",
        ]
        return {
            name: (self._session_dir / f"{name}.jsonl").open("a", encoding="utf-8", buffering=1)
            for name in names
        }

    def _write_session_metadata(self) -> None:
        metadata = {
            "node": self.get_name(),
            "started_at": self._now_iso(),
            "log_dir": str(self._session_dir),
            "parameters": {
                "can_port": self.can_port,
                "motor_max_speed": self.motor_max_speed,
                "drive_max_torque": self.drive_max_torque,
                "drive_acceleration_limit": self.drive_acceleration_limit,
                "steer_max_torque": self.steer_max_torque,
                "steer_velocity_limit": self.steer_velocity_limit,
                "steer_acceleration_limit": self.steer_acceleration_limit,
                "controller_deadzone": self.controller_deadzone,
                "command_timeout_s": self.command_timeout_s,
                "refresh_rate_hz": self.refresh_rate_hz,
                "autonomy_priority": self.autonomy_priority,
                "state_poll_hz": self.state_poll_hz,
                "session_log_root": str(self.session_log_root),
            },
            "drive_motor_ids": DRIVE_IDS,
            "steer_motor_ids": STEER_IDS,
        }
        (self._session_dir / "session_metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True),
            encoding="utf-8",
        )

    def _log_event(self, stream: str, payload: dict[str, object]) -> None:
        handle = self._log_files.get(stream)
        if handle is None:
            return
        with self._log_lock:
            self._log_sequence += 1
            record = {
                "seq": self._log_sequence,
                "time": self._now_iso(),
                "uptime_s": round(time.time() - self._session_started_at, 6),
                **payload,
            }
            handle.write(json.dumps(record, sort_keys=True, default=self._json_default) + "\n")

    def _close_log_files(self) -> None:
        with self._log_lock:
            for handle in self._log_files.values():
                handle.close()
            self._log_files.clear()

    @staticmethod
    def _json_default(value):
        if isinstance(value, Path):
            return str(value)
        return repr(value)

    @staticmethod
    def _now_iso() -> str:
        return datetime.now().astimezone().isoformat()

    @staticmethod
    def _ros_time_to_dict(stamp) -> dict[str, int]:
        return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}

    @staticmethod
    def _clean_number(value):
        if value is None:
            return None
        if isinstance(value, float) and math.isnan(value):
            return "nan"
        if isinstance(value, (float, int)):
            return float(value)
        return value

    def _clean_command_specs(self, specs: list[dict[str, object]]) -> list[dict[str, object]]:
        cleaned = []
        for spec in specs:
            cleaned.append(
                {
                    key: self._clean_number(value) if key in {"position", "velocity"} else value
                    for key, value in spec.items()
                }
            )
        return cleaned

    @staticmethod
    def _trigger_to_speed_scale(trigger_byte: int) -> float:
        value = int(trigger_byte)
        if value < TRIGGER_DEADBAND:
            return 0.0
        if value > TRIGGER_MAX:
            value = TRIGGER_MAX
        return value / TRIGGER_MAX

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float = 0.1) -> float:
        return 0.0 if abs(value) < deadzone else value

    @staticmethod
    def _clamp(value: float, limit: float = 1.0) -> float:
        return max(-limit, min(limit, value))

    def _wheel_angles_and_speeds(
        self, vx: float, vy: float, omega: float, length: float, width: float
    ) -> tuple[float, float, float, float, float, float, float, float]:
        radius = math.sqrt(length**2 + width**2)

        vx = round(vx, 3)
        vy = round(vy, 3)
        omega = round(omega, 3)

        a = vy - omega * (length / radius)
        b = vy + omega * (length / radius)
        c = vx - omega * (width / radius)
        d = vx + omega * (width / radius)

        ws1 = math.sqrt(b**2 + c**2)
        ws2 = math.sqrt(b**2 + d**2)
        ws3 = math.sqrt(a**2 + d**2)
        ws4 = math.sqrt(a**2 + c**2)

        wa1 = math.atan2(b, c) * 180.0 / math.pi
        wa2 = math.atan2(b, d) * 180.0 / math.pi
        wa3 = math.atan2(a, d) * 180.0 / math.pi
        wa4 = math.atan2(a, c) * 180.0 / math.pi

        max_speed = max(ws1, ws2, ws3, ws4, 1.0)
        ws1 /= max_speed
        ws2 /= max_speed
        ws3 /= max_speed
        ws4 /= max_speed

        wa1 = (round(wa1, 3) / 360.0) * SWERVE_RATIO
        wa2 = (round(wa2, 3) / 360.0) * SWERVE_RATIO
        wa3 = (round(wa3, 3) / 360.0) * SWERVE_RATIO
        wa4 = (round(wa4, 3) / 360.0) * SWERVE_RATIO

        return ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4

    def destroy_node(self):
        self.get_logger().info("Shutting down zenny_drives_node")
        self._shutdown.set()
        try:
            if self._worker.is_alive():
                self._worker.join(timeout=1.0)
        except RuntimeError:
            pass

        if self._transport and self._controllers:
            try:
                asyncio.run(self._async_stop_all())
            except Exception as exc:  # pragma: no cover - hardware path
                self.get_logger().warn(f"Final drive stop failed: {exc!r}")
                self._log_event("worker_errors", {"stage": "destroy_node", "error": repr(exc)})

        self._close_log_files()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZennyDrivesNode()
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
    main()
