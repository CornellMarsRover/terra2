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
STEER_IDS = [5, 6, 7, 8]
STEER_DEGREES_TO_POSITION = 50.0 / 360.0
STEER_DIRECTION_SIGN = 1.0

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
    6: 1.0,  # BL steer
    7: 1.0,  # FR steer
    8: 1.0,  # BR steer
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


async def command_steer_motors_async(
    port_arg: str,
    timeout_s: float,
    steer_angle_deg: float,
    max_torque: float,
    velocity_limit: float,
    watchdog_timeout: float,
):
    port, controllers = make_transport_and_controllers(port_arg, STEER_IDS)
    target_position = steer_angle_deg * STEER_DEGREES_TO_POSITION

    for motor_id in STEER_IDS:
        _label, ctrl = controllers[motor_id]
        await asyncio.wait_for(
            ctrl.set_position(
                position=STEER_SIGN[motor_id] * target_position,
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
        self.rows: dict[int, dict[str, tk.StringVar]] = {}
        self.task_queue: queue.Queue = queue.Queue()
        self.result_queue: queue.Queue = queue.Queue()
        self.motion_refresh_s = 0.10
        self.latched_motion_direction = 0.0
        self.latched_motion_label = "stopped"

        self._build_layout()

        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()
        self.root.after(100, self._poll_results)
        self.queue_query_drives()

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

        buttons = ttk.Frame(outer, padding=(0, 10, 0, 0))
        buttons.pack(fill="x")

        self.forward_button = ttk.Button(buttons, text="Toggle Forward")
        self.forward_button.pack(side="left", padx=(0, 6))
        self.reverse_button = ttk.Button(buttons, text="Toggle Reverse")
        self.reverse_button.pack(side="left", padx=6)
        self.left_button = ttk.Button(buttons, text="Steer Left")
        self.left_button.pack(side="left", padx=6)
        self.right_button = ttk.Button(buttons, text="Steer Right")
        self.right_button.pack(side="left", padx=6)
        ttk.Button(buttons, text="Center Steering", command=self.center_steering).pack(
            side="left",
            padx=6,
        )
        ttk.Button(buttons, text="Stop All", command=self.queue_stop_all).pack(
            side="left",
            padx=6,
        )
        ttk.Button(buttons, text="Query Drives", command=self.queue_query_drives).pack(
            side="left",
            padx=6,
        )

        ttk.Label(
            outer,
            text="Keep the rover supported or wheels off the ground before bump tests.",
            padding=(0, 10, 0, 4),
        ).pack(anchor="w")

        table = ttk.LabelFrame(outer, text="Drive Motor Status", padding=8)
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

        log_frame = ttk.LabelFrame(outer, text="Log", padding=8)
        log_frame.pack(fill="both", expand=True, pady=(10, 0))

        self.log_text = tk.Text(log_frame, height=12, wrap="word")
        self.log_text.pack(fill="both", expand=True)
        self.log_text.configure(state="disabled")

        ttk.Label(outer, textvariable=self.status_var, padding=(0, 8, 0, 0)).pack(anchor="w")

        self.forward_button.bind("<ButtonPress-1>", lambda _event: self.toggle_motion(1.0, "forward"))
        self.reverse_button.bind("<ButtonPress-1>", lambda _event: self.toggle_motion(-1.0, "reverse"))
        self.left_button.bind("<ButtonPress-1>", lambda _event: self.start_steering(-1.0, "left"))
        self.right_button.bind("<ButtonPress-1>", lambda _event: self.start_steering(1.0, "right"))

        self.root.bind("<KeyPress-w>", lambda _event: self.toggle_motion(1.0, "forward"))
        self.root.bind("<KeyPress-W>", lambda _event: self.toggle_motion(1.0, "forward"))
        self.root.bind("<KeyPress-s>", lambda _event: self.toggle_motion(-1.0, "reverse"))
        self.root.bind("<KeyPress-S>", lambda _event: self.toggle_motion(-1.0, "reverse"))
        self.root.bind("<KeyPress-a>", lambda _event: self.start_steering(-1.0, "left"))
        self.root.bind("<KeyPress-A>", lambda _event: self.start_steering(-1.0, "left"))
        self.root.bind("<KeyPress-d>", lambda _event: self.start_steering(1.0, "right"))
        self.root.bind("<KeyPress-D>", lambda _event: self.start_steering(1.0, "right"))
        self.root.bind("<space>", lambda _event: self.stop_motion())
        self.root.focus_force()

    def _append_log(self, message: str):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{time.strftime('%H:%M:%S')}  {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

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

    def toggle_motion(self, direction: float, label: str):
        try:
            settings = self._get_settings()
        except ValueError as exc:
            self.status_var.set(f"Invalid numeric input: {exc}")
            return

        if self.latched_motion_direction == direction:
            self.latched_motion_direction = 0.0
            self.latched_motion_label = "stopped"
            self.status_var.set(f"Stopping {label}")
            self.task_queue.put(("stop_motion", settings))
            return

        self.latched_motion_direction = direction
        self.latched_motion_label = label
        self.status_var.set(f"Driving {label}")
        self.task_queue.put(("start_motion", label, direction, settings))

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

        self.latched_motion_direction = 0.0
        self.latched_motion_label = "stopped"
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
                        )
                    )
                    self.result_queue.put(("steering_started", label, port, settings))
                elif task[0] == "center_steering":
                    settings = task[1]
                    port = asyncio.run(
                        command_steer_motors_async(
                            port_arg=settings["port"],
                            timeout_s=settings["timeout"],
                            steer_angle_deg=0.0,
                            max_torque=settings["steer_torque"],
                            velocity_limit=settings["steer_vel_limit"],
                            watchdog_timeout=settings["watchdog"],
                        )
                    )
                    active_steering = None
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
                self.status_var.set(f"Error: {item[1]}")
                self._append_log(f"ERROR {item[1]}")
            elif kind == "query_done":
                _kind, port, results = item
                for result in results:
                    self._set_row_from_query(result.motor_id, result)
                self.status_var.set(f"Queried drives on {port or 'default transport'}")
                self._append_log("Drive query completed")
            elif kind == "stop_done":
                _kind, port, statuses = item
                self.latched_motion_direction = 0.0
                self.latched_motion_label = "stopped"
                self.status_var.set(f"Stop all sent on {port or 'default transport'}")
                self._append_log(f"Stop statuses: {statuses}")
            elif kind == "motion_started":
                _kind, label, port, settings = item
                self.status_var.set(f"Driving {label} on {port or 'default transport'}")
                self._append_log(
                    f"{label.title()} motion started: speed={settings['drive_rps']:.2f} rev/s, "
                    f"watchdog={settings['watchdog']:.2f} s, torque={settings['max_torque']:.2f}"
                )
            elif kind == "motion_active":
                _kind, label, port, _settings = item
                self.status_var.set(f"Driving {label} on {port or 'default transport'}")
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
                self.status_var.set(f"Steering centered on {port or 'default transport'}")
                self._append_log("Steering returned to center")
            elif kind == "motion_stopped":
                _kind, port, statuses = item
                self.latched_motion_direction = 0.0
                self.latched_motion_label = "stopped"
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
