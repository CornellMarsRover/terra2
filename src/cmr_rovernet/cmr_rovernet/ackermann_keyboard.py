#!/usr/bin/env python3
"""
Direct moteus GUI for straight drive bump testing.

This uses the same style of direct drive command as the moteus debug tool,
but applies it to all four drive motors together so we can verify that the
rover moves forward and backward with stronger, tunable commands.
"""

from __future__ import annotations

import asyncio
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

from cmr_rovernet.moteus_drive_gui import (
    EXPECTED_MODULES,
    fmt,
    make_transport_and_controllers,
    query_one,
    read_value,
    stop_compat,
)


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
    5: 1.0,   # FL steer
    7: 1.0,   # FR steer
    6: 1.0,   # BL steer
    8: 1.0,   # BR steer
}

SPIN_STEER_SIGN = {
    5: 1.0,
    6: -1.0,
    7: -1.0,
    8: 1.0,
}


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


def build_custom_steer_targets(
    angle_deg_by_motor: dict[int, float],
    center_offsets: dict[int, float],
) -> dict[int, float]:
    targets: dict[int, float] = {}
    for motor_id in STEER_IDS:
        steer_delta = angle_deg_by_motor[motor_id] * STEER_DEGREES_TO_POSITION
        targets[motor_id] = center_offsets[motor_id] + (
            STEER_SIGN[motor_id] * steer_delta
        )
    return targets


def build_spin_steer_targets(
    spin_angle_deg: float,
    center_offsets: dict[int, float],
) -> dict[int, float]:
    return build_custom_steer_targets(
        {
            motor_id: SPIN_STEER_SIGN[motor_id] * spin_angle_deg
            for motor_id in STEER_IDS
        },
        center_offsets,
    )


async def command_custom_steer_targets_async(
    port_arg: str,
    timeout_s: float,
    target_positions: dict[int, float],
    max_torque: float,
    velocity_limit: float,
    watchdog_timeout: float,
):
    port, controllers = make_transport_and_controllers(port_arg, STEER_IDS)
    for motor_id in STEER_IDS:
        _label, ctrl = controllers[motor_id]
        await asyncio.wait_for(
            ctrl.set_position(
                position=target_positions[motor_id],
                velocity_limit=velocity_limit,
                maximum_torque=max_torque,
                watchdog_timeout=max(0.25, watchdog_timeout),
            ),
            timeout=timeout_s,
        )
    return port


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
        self.current_spin_button_label = "Center"
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

        self.direction_buttons["Spin Orient"] = ttk.Button(
            buttons_frame,
            text="Spin Orient",
            command=self.orient_spin_steering,
        )
        self.direction_buttons["Spin Orient"].grid(row=4, column=1, padx=4, pady=4, sticky="ew")

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
            text="Use buttons or keyboard: W=North, S=South, A=West, D=East, J=Orbit East, L=Orbit West, Q=Spin Orient.",
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
        for key in ["w", "W", "s", "S", "a", "A", "d", "D", "j", "J", "l", "L", "q", "Q"]:
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

        if "q" in self.keyboard_pressed:
            if self.current_spin_button_label != "Spin Orient":
                self.orient_spin_steering()

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
                or label == self.current_spin_button_label
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
        self.current_orbit_button_label = "Center"
        self.current_orbit_direction = 0.0
        self.current_spin_button_label = "Center"
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

    def _queue_spin_orientation(self, settings: dict[str, float | str], label: str):
        target_positions = build_spin_steer_targets(
            float(settings["steer_angle_deg"]),
            self.steer_center_offsets,
        )
        self.task_queue.put(("start_custom_steering", label, target_positions, settings))

    def orient_spin_steering(self):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        self.current_drive_button_label = "Center"
        self.current_orbit_button_label = "Center"
        self.current_spin_button_label = "Spin Orient"
        self.current_drive_direction = 0.0
        self.current_orbit_direction = 0.0
        self.current_steer_button_label = "Spin"
        self.current_steer_direction = 0.0
        self.current_steer_angle_deg = float(settings["steer_angle_deg"])
        self.control_state_var.set("Spin Orient")
        self.control_debug_var.set(
            f"spin oriented angle={self.current_steer_angle_deg:.1f}"
        )
        self._refresh_direction_buttons()
        self.status_var.set("Orienting wheels for spin")
        self._queue_spin_orientation(settings, "spin orientation")

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
        self.current_orbit_button_label = "Center"
        self.current_spin_button_label = "Center"
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
        self.current_spin_button_label = "Center"
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
        self.current_spin_button_label = "Center"
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
        self.current_spin_button_label = "Center"
        self.current_drive_direction = 0.0
        self.control_state_var.set("Center")
        self.control_debug_var.set("orbit centered")
        self._refresh_direction_buttons()
        self.task_queue.put(("stop_motion", settings))

    def reset_direction_toggle(self):
        self.current_drive_button_label = "Center"
        self.current_orbit_button_label = "Center"
        self.current_spin_button_label = "Center"
        self.current_steer_button_label = "Center"
        self.current_orbit_direction = 0.0
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
                        steer_mode, label, steer_value, settings = active_steering
                        if steer_mode == "custom":
                            port = asyncio.run(
                                command_custom_steer_targets_async(
                                    port_arg=settings["port"],
                                    timeout_s=settings["timeout"],
                                    target_positions=steer_value,
                                    max_torque=settings["steer_torque"],
                                    velocity_limit=settings["steer_vel_limit"],
                                    watchdog_timeout=settings["watchdog"],
                                )
                            )
                        else:
                            port = asyncio.run(
                                command_steer_motors_async(
                                    port_arg=settings["port"],
                                    timeout_s=settings["timeout"],
                                    steer_angle_deg=steer_value,
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
                    active_steering = ("uniform", label, steer_angle_deg, settings)
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
                elif task[0] == "start_custom_steering":
                    _kind, label, target_positions, settings = task
                    active_steering = ("custom", label, target_positions, settings)
                    port = asyncio.run(
                        command_custom_steer_targets_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                            target_positions=target_positions,
                            max_torque=settings["steer_torque"],
                            velocity_limit=settings["steer_vel_limit"],
                            watchdog_timeout=settings["watchdog"],
                        )
                    )
                    self.result_queue.put(("steering_started", label, port, settings))
                elif task[0] == "center_steering":
                    settings = task[1]
                    active_steering = ("uniform", "center", 0.0, settings)
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
                self.current_spin_button_label = "Center"
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
                if label == "spin orientation":
                    self._append_log(
                        f"Steer {label}: X-orientation angle={settings['steer_angle_deg']:.1f} deg, "
                        f"vel_limit={settings['steer_vel_limit']:.2f}, torque={settings['steer_torque']:.2f}"
                    )
                else:
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


if __name__ == "__main__":
    main()
