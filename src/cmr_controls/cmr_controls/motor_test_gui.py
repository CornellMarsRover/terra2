#!/usr/bin/env python3

import asyncio
import concurrent.futures
import glob
import math
import os
import sys
import threading
import tkinter as tk
from tkinter import ttk

import moteus


# This mapping is intentionally kept identical to swerve_controller_node.py.
# 1-4 are drive motors, 5-8 are swerve motors.
MOTOR_MAP = {
    1: "FL_DRIVE",
    2: "BL_DRIVE",
    3: "FR_DRIVE",
    4: "BR_DRIVE",
    5: "FL_SWERVE",
    6: "BL_SWERVE",
    7: "FR_SWERVE",
    8: "BR_SWERVE",
}


class MotorTestGUI:
    def __init__(self) -> None:
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.loop_thread.start()

        self.root = tk.Tk()
        self.root.title("CMR Moteus Motor Test")
        self.root.geometry("820x520")
        self.root.minsize(760, 500)

        self.transport = None
        self.servos = {}
        self.connected = False

        self.velocity_var = tk.DoubleVar(value=1.0)
        self.duration_var = tk.DoubleVar(value=0.35)
        self.accel_var = tk.DoubleVar(value=80.0)
        self.torque_var = tk.DoubleVar(value=1.5)
        self.velocity_limit_var = tk.DoubleVar(value=10.0)
        self.path_var = tk.StringVar(value="")
        self.path_info_var = tk.StringVar(value="Detected paths: (not scanned yet)")

        self.status_var = tk.StringVar(value="Disconnected.")
        self.motor_buttons = []
        self.stop_all_button = None
        self.connect_button = None
        self._build_ui()
        self._try_initialize_moteus()

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, padding=12)
        container.pack(fill=tk.BOTH, expand=True)

        title = ttk.Label(
            container,
            text="Moteus Per-Motor Short Movement Test",
            font=("TkDefaultFont", 14, "bold"),
        )
        title.pack(anchor="w", pady=(0, 8))

        info_text = (
            "ID map matches cmr_controls/swerve_controller_node.py:\n"
            "1 FL_DRIVE, 2 BL_DRIVE, 3 FR_DRIVE, 4 BR_DRIVE, "
            "5 FL_SWERVE, 6 BL_SWERVE, 7 FR_SWERVE, 8 BR_SWERVE"
        )
        ttk.Label(container, text=info_text).pack(anchor="w", pady=(0, 10))

        connection = ttk.LabelFrame(container, text="Connection", padding=10)
        connection.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(connection, text="fdcanusb path (optional):").grid(
            row=0, column=0, padx=6, pady=6, sticky="w"
        )
        ttk.Entry(connection, textvariable=self.path_var).grid(
            row=0, column=1, padx=6, pady=6, sticky="ew"
        )
        self.connect_button = tk.Button(
            connection,
            text="Disconnected",
            command=self._try_initialize_moteus,
            bg="#b91c1c",
            fg="white",
            activebackground="#991b1b",
            activeforeground="white",
            relief=tk.RAISED,
            bd=1,
        )
        self.connect_button.grid(row=0, column=2, padx=6, pady=6, sticky="ew")
        ttk.Button(connection, text="Refresh Paths", command=self._refresh_path_info).grid(
            row=0, column=3, padx=6, pady=6, sticky="ew"
        )
        ttk.Label(
            connection,
            textvariable=self.path_info_var,
            wraplength=700,
            justify=tk.LEFT,
        ).grid(row=1, column=0, columnspan=4, padx=6, pady=(0, 4), sticky="w")
        connection.columnconfigure(1, weight=1)

        controls = ttk.LabelFrame(container, text="Test Command Parameters", padding=10)
        controls.pack(fill=tk.X, pady=(0, 10))

        self._add_labeled_entry(controls, "Velocity (rev/s)", self.velocity_var, 0, 0)
        self._add_labeled_entry(controls, "Duration (s)", self.duration_var, 0, 2)
        self._add_labeled_entry(controls, "Accel Limit", self.accel_var, 1, 0)
        self._add_labeled_entry(controls, "Torque Limit", self.torque_var, 1, 2)
        self._add_labeled_entry(controls, "Velocity Limit", self.velocity_limit_var, 2, 0)

        self.stop_all_button = ttk.Button(
            controls,
            text="Stop All Motors",
            command=self._on_stop_all_clicked,
        )
        self.stop_all_button.grid(row=2, column=2, padx=6, pady=6, sticky="ew")

        for col in range(4):
            controls.columnconfigure(col, weight=1)

        motors_frame = ttk.LabelFrame(container, text="Motors (One Button Per ID)", padding=10)
        motors_frame.pack(fill=tk.BOTH, expand=True)

        row = 0
        col = 0
        for motor_id in sorted(MOTOR_MAP):
            label = MOTOR_MAP[motor_id]
            button_text = f"Test ID {motor_id} ({label})"
            button = ttk.Button(
                motors_frame,
                text=button_text,
                command=lambda mid=motor_id: self._on_motor_test_clicked(mid),
            )
            button.grid(row=row, column=col, padx=6, pady=6, sticky="ew")
            self.motor_buttons.append(button)
            col += 1
            if col > 1:
                col = 0
                row += 1

        motors_frame.columnconfigure(0, weight=1)
        motors_frame.columnconfigure(1, weight=1)

        status_box = ttk.LabelFrame(container, text="Status", padding=8)
        status_box.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(status_box, textvariable=self.status_var).pack(anchor="w")

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._set_controls_enabled(False)
        self._set_connection_button_state(False)
        self._refresh_path_info()

    def _add_labeled_entry(self, parent, label, variable, row, column) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=column, padx=6, pady=6, sticky="w")
        ttk.Entry(parent, textvariable=variable, width=10).grid(
            row=row, column=column + 1, padx=6, pady=6, sticky="ew"
        )

    def _set_status(self, text: str) -> None:
        if hasattr(self, "status_var"):
            self.status_var.set(text)
        else:
            print(text)

    def _run_event_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _run_blocking(self, coro):
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return future.result(timeout=20)

    async def _async_initialize_moteus(self, requested_path: str = "") -> None:
        fdcanusb_path = requested_path.strip() if requested_path else self._detect_fdcanusb_path()
        self.transport = moteus.Fdcanusb(path=fdcanusb_path)
        self.servos = {
            motor_id: moteus.Controller(id=motor_id, transport=self.transport)
            for motor_id in MOTOR_MAP
        }
        await self.transport.cycle([servo.make_stop() for servo in self.servos.values()])
        self._connected_path = fdcanusb_path

    def _detect_fdcanusb_path(self) -> str:
        candidates = self._probe_fdcanusb_candidates()
        if candidates:
            return candidates[0]
        raise RuntimeError(
            "No fdcanusb device detected. Connect the adapter or enter a device path manually."
        )

    def _probe_fdcanusb_candidates(self):
        candidates = []
        seen = set()

        def add_path(path):
            if path and path not in seen:
                seen.add(path)
                candidates.append(path)

        if os.path.exists("/dev/fdcanusb"):
            add_path("/dev/fdcanusb")

        for p in sorted(glob.glob("/dev/serial/by-id/*fdcanusb*")):
            add_path(p)

        if sys.platform != "win32":
            for p in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
                add_path(p)

        try:
            add_path(moteus.Fdcanusb.detect_fdcanusb())
        except Exception:
            pass

        return candidates

    def _refresh_path_info(self) -> None:
        candidates = self._probe_fdcanusb_candidates()
        if candidates:
            self.path_info_var.set("Detected paths: " + ", ".join(candidates))
            if not self.path_var.get().strip():
                self.path_var.set(candidates[0])
        else:
            self.path_info_var.set("Detected paths: none")

    def _set_controls_enabled(self, enabled: bool) -> None:
        state = tk.NORMAL if enabled else tk.DISABLED
        if self.stop_all_button is not None:
            self.stop_all_button.config(state=state)
        for button in self.motor_buttons:
            button.config(state=state)

    def _set_connection_button_state(self, connected: bool) -> None:
        if self.connect_button is None:
            return
        if connected:
            self.connect_button.config(
                text="Connected",
                bg="#15803d",
                fg="white",
                activebackground="#166534",
                activeforeground="white",
            )
        else:
            self.connect_button.config(
                text="Disconnected",
                bg="#b91c1c",
                fg="white",
                activebackground="#991b1b",
                activeforeground="white",
            )

    def _try_initialize_moteus(self) -> None:
        self._refresh_path_info()
        self._set_status("Initializing moteus transport...")
        requested_path = self.path_var.get()
        future = asyncio.run_coroutine_threadsafe(
            self._async_initialize_moteus(requested_path), self.loop
        )
        future.add_done_callback(lambda f: self.root.after(0, self._on_connect_done, f))

    def _on_connect_done(self, future: concurrent.futures.Future) -> None:
        try:
            future.result()
            self.connected = True
            self._set_controls_enabled(True)
            self._set_connection_button_state(True)
            self._set_status(f"Connected on {self._connected_path}.")
        except Exception as exc:
            self.connected = False
            self._set_controls_enabled(False)
            self._set_connection_button_state(False)
            self._set_status(f"Connection failed: {type(exc).__name__}: {exc}")

    def _read_params(self):
        return {
            "velocity": float(self.velocity_var.get()),
            "duration": max(0.05, float(self.duration_var.get())),
            "accel_limit": float(self.accel_var.get()),
            "maximum_torque": float(self.torque_var.get()),
            "velocity_limit": float(self.velocity_limit_var.get()),
        }

    def _on_motor_test_clicked(self, motor_id: int) -> None:
        if not self.connected:
            self._set_status("Not connected. Press Connect / Reconnect first.")
            return
        try:
            params = self._read_params()
        except ValueError:
            self._set_status("Invalid numeric input in test parameters.")
            return

        label = MOTOR_MAP[motor_id]
        self._set_status(f"Testing ID {motor_id} ({label})...")

        future = asyncio.run_coroutine_threadsafe(
            self._async_pulse_motor(motor_id, params), self.loop
        )
        future.add_done_callback(
            lambda f, mid=motor_id, name=label: self._on_test_done(mid, name, f)
        )

    def _on_test_done(
        self, motor_id: int, label: str, future: concurrent.futures.Future
    ) -> None:
        try:
            future.result()
            msg = f"Completed test: ID {motor_id} ({label})."
        except Exception as exc:
            msg = f"Test failed for ID {motor_id} ({label}): {type(exc).__name__}: {exc}"
        self.root.after(0, lambda: self._set_status(msg))

    async def _async_pulse_motor(self, motor_id: int, params) -> None:
        servo = self.servos[motor_id]

        command = servo.make_position(
            position=math.nan,
            velocity=params["velocity"],
            accel_limit=params["accel_limit"],
            velocity_limit=params["velocity_limit"],
            maximum_torque=params["maximum_torque"],
            query=False,
        )
        await self.transport.cycle([command])
        await asyncio.sleep(params["duration"])
        await self.transport.cycle([servo.make_stop(query=False)])

    def _on_stop_all_clicked(self) -> None:
        if not self.connected:
            self._set_status("Not connected. Press Connect / Reconnect first.")
            return
        self._set_status("Sending stop to all motors...")
        future = asyncio.run_coroutine_threadsafe(self._async_stop_all(), self.loop)
        future.add_done_callback(lambda f: self.root.after(0, self._on_stop_done, f))

    def _on_stop_done(self, future: concurrent.futures.Future) -> None:
        try:
            future.result()
            self._set_status("All motors stopped.")
        except Exception as exc:
            self._set_status(f"Stop all failed: {type(exc).__name__}: {exc}")

    async def _async_stop_all(self) -> None:
        await self.transport.cycle([servo.make_stop(query=False) for servo in self.servos.values()])

    def _on_close(self) -> None:
        try:
            self._run_blocking(self._async_stop_all())
        except Exception:
            pass
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    app = MotorTestGUI()
    app.run()


if __name__ == "__main__":
    main()
