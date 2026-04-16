#!/usr/bin/env python3
"""
Tk GUI for testing individual moteus drive and steer controllers.

This is intended as a hands-on diagnostic tool:
- Query all motors or a single motor.
- Test one motor at a time.
- Adjust drive velocity/hold time/torque.
- Adjust steer delta/hold time/torque/velocity limit.

For drive motors (IDs 1-4), the test uses velocity mode.
For steer motors (IDs 5-8), the test commands a relative position delta.
"""

from __future__ import annotations

import asyncio
import glob
import math
import os
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk
from typing import Any

import moteus


DEFAULT_PORT = "/dev/ttyACM0"

EXPECTED_MODULES = {
    1: "FL drive",
    2: "BL drive",
    3: "FR drive",
    4: "BR drive",
    5: "FL steer",
    6: "BL steer",
    7: "FR steer",
    8: "BR steer",
}


@dataclass
class QueryResult:
    motor_id: int
    label: str
    ok: bool
    error: str | None
    values: dict[str, Any]


def reg(name: str):
    return getattr(moteus.Register, name, None)


REGISTER_NAMES = {
    reg_name: reg(reg_name)
    for reg_name in [
        "MODE",
        "FAULT",
        "POSITION",
        "VELOCITY",
        "TORQUE",
        "VOLTAGE",
        "TEMPERATURE",
        "MOTOR_TEMPERATURE",
        "Q_CURRENT",
        "D_CURRENT",
        "POWER",
    ]
    if reg(reg_name) is not None
}


def read_value(result, register_name: str):
    register = REGISTER_NAMES.get(register_name)
    if register is None:
        return None
    return result.values.get(register)


def fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        if math.isnan(value):
            return "nan"
        return f"{value:.{digits}f}"
    return str(value)


def make_query_resolution() -> moteus.QueryResolution:
    qr = moteus.QueryResolution()
    for field, resolution in [
        ("mode", moteus.INT8),
        ("fault", moteus.INT8),
        ("position", moteus.F32),
        ("velocity", moteus.F32),
        ("torque", moteus.F32),
        ("voltage", moteus.F32),
        ("temperature", moteus.F32),
        ("motor_temperature", moteus.F32),
        ("q_current", moteus.F32),
        ("d_current", moteus.F32),
        ("power", moteus.F32),
    ]:
        if hasattr(qr, field):
            setattr(qr, field, resolution)
    return qr


def find_can_ports() -> list[str]:
    candidates: list[str] = []
    patterns = [
        "/dev/serial/by-id/*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ]
    for pattern in patterns:
        for candidate in sorted(glob.glob(pattern)):
            real_path = os.path.realpath(candidate)
            if real_path not in candidates:
                candidates.append(real_path)
    return candidates


def resolve_port(port_arg: str | None) -> str | None:
    if port_arg in (None, ""):
        return None
    if port_arg != "auto":
        return port_arg

    candidates = find_can_ports()
    if not candidates:
        raise FileNotFoundError(
            "Could not find a CAN USB adapter. Checked /dev/serial/by-id/*, "
            "/dev/ttyACM*, and /dev/ttyUSB*."
        )
    return candidates[0]


def make_controller(motor_id: int, transport, qr: moteus.QueryResolution):
    kwargs = {"id": motor_id, "query_resolution": qr}
    if transport is not None:
        kwargs["transport"] = transport
    return moteus.Controller(**kwargs)


async def query_one(ctrl, motor_id: int, label: str, timeout_s: float) -> QueryResult:
    try:
        result = await asyncio.wait_for(ctrl.query(), timeout=timeout_s)
    except Exception as exc:
        return QueryResult(motor_id, label, False, repr(exc), {})

    values = {name: read_value(result, name) for name in REGISTER_NAMES}
    return QueryResult(motor_id, label, True, None, values)


async def stop_compat(ctrl, clear_faults: bool = False):
    if clear_faults:
        try:
            await ctrl.set_stop(clear_faults=True)
            return "stop/clear sent"
        except TypeError as exc:
            if "clear_faults" not in str(exc):
                raise

    await ctrl.set_stop()
    if clear_faults:
        return "stop sent; clear_faults=True unsupported"
    return "stop sent"


def make_transport_and_controllers(port_arg: str, ids: list[int]):
    qr = make_query_resolution()
    port = resolve_port(port_arg)
    transport = moteus.Fdcanusb(port) if port else None
    controllers = {
        motor_id: (
            EXPECTED_MODULES.get(motor_id, "unknown"),
            make_controller(motor_id, transport, qr),
        )
        for motor_id in ids
    }
    return port, controllers


async def query_ids_async(port_arg: str, ids: list[int], timeout_s: float) -> tuple[str | None, list[QueryResult]]:
    port, controllers = make_transport_and_controllers(port_arg, ids)
    results: list[QueryResult] = []
    for motor_id in ids:
        label, ctrl = controllers[motor_id]
        results.append(await query_one(ctrl, motor_id, label, timeout_s))
    return port, results


async def stop_ids_async(port_arg: str, ids: list[int], timeout_s: float) -> tuple[str | None, dict[int, str]]:
    port, controllers = make_transport_and_controllers(port_arg, ids)
    statuses: dict[int, str] = {}
    for motor_id in ids:
        _label, ctrl = controllers[motor_id]
        try:
            statuses[motor_id] = await asyncio.wait_for(stop_compat(ctrl), timeout=timeout_s)
        except Exception as exc:
            statuses[motor_id] = repr(exc)
    return port, statuses


async def test_motor_async(
    port_arg: str,
    motor_id: int,
    timeout_s: float,
    drive_rps: float,
    drive_hold_time: float,
    drive_torque: float,
    steer_delta: float,
    steer_hold_time: float,
    steer_torque: float,
    steer_velocity_limit: float,
) -> tuple[str | None, QueryResult, QueryResult]:
    port, controllers = make_transport_and_controllers(port_arg, [motor_id])
    label, ctrl = controllers[motor_id]

    before = await query_one(ctrl, motor_id, label, timeout_s)
    if not before.ok:
        return port, before, before

    before_pos = before.values.get("POSITION")
    is_drive = 1 <= motor_id <= 4

    try:
        if is_drive:
            await asyncio.wait_for(
                ctrl.set_position(
                    position=math.nan,
                    velocity=drive_rps,
                    maximum_torque=drive_torque,
                    accel_limit=5.0,
                    watchdog_timeout=max(0.25, drive_hold_time + 0.25),
                ),
                timeout=timeout_s,
            )
            await asyncio.sleep(drive_hold_time)
        else:
            target = (before_pos or 0.0) + steer_delta
            await asyncio.wait_for(
                ctrl.set_position(
                    position=target,
                    velocity_limit=steer_velocity_limit,
                    maximum_torque=steer_torque,
                    watchdog_timeout=max(0.5, steer_hold_time + 0.25),
                ),
                timeout=timeout_s,
            )
            await asyncio.sleep(steer_hold_time)
    finally:
        try:
            await asyncio.wait_for(stop_compat(ctrl), timeout=timeout_s)
        except Exception:
            pass

    after = await query_one(ctrl, motor_id, label, timeout_s)
    return port, before, after


class MoteusGuiApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("CMR Moteus Drive Diagnostics")
        self.root.geometry("1080x680")

        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.timeout_var = tk.StringVar(value="0.20")

        self.drive_rps_var = tk.StringVar(value="4.0")
        self.drive_hold_var = tk.StringVar(value="1.5")
        self.drive_torque_var = tk.StringVar(value="2.0")

        self.steer_delta_var = tk.StringVar(value="3.0")
        self.steer_hold_var = tk.StringVar(value="2.0")
        self.steer_torque_var = tk.StringVar(value="3.0")
        self.steer_vel_limit_var = tk.StringVar(value="3.0")

        self.status_var = tk.StringVar(value="Ready")
        self.task_queue: queue.Queue = queue.Queue()
        self.result_queue: queue.Queue = queue.Queue()
        self.rows: dict[int, dict[str, tk.StringVar]] = {}

        self._build_layout()

        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()
        self.root.after(100, self._poll_results)
        self.queue_query_all()

    def _build_layout(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        port_frame = ttk.LabelFrame(top, text="Connection", padding=8)
        port_frame.pack(fill="x")

        ttk.Label(port_frame, text="Port").grid(row=0, column=0, sticky="w")
        ttk.Entry(port_frame, textvariable=self.port_var, width=32).grid(row=0, column=1, sticky="w", padx=4)
        ttk.Label(port_frame, text="Use 'auto' or /dev/ttyACM0").grid(row=0, column=2, sticky="w", padx=4)
        ttk.Label(port_frame, text="Timeout (s)").grid(row=0, column=3, sticky="w", padx=(16, 0))
        ttk.Entry(port_frame, textvariable=self.timeout_var, width=8).grid(row=0, column=4, sticky="w", padx=4)
        ttk.Button(port_frame, text="Refresh All", command=self.queue_query_all).grid(row=0, column=5, padx=(16, 4))
        ttk.Button(port_frame, text="Stop All", command=self.queue_stop_all).grid(row=0, column=6, padx=4)

        motion = ttk.Frame(self.root, padding=(10, 4))
        motion.pack(fill="x")

        drive_frame = ttk.LabelFrame(motion, text="Drive Test Settings", padding=8)
        drive_frame.pack(side="left", fill="x", expand=True, padx=(0, 6))
        ttk.Label(drive_frame, text="Velocity (rev/s)").grid(row=0, column=0, sticky="w")
        ttk.Entry(drive_frame, textvariable=self.drive_rps_var, width=10).grid(row=0, column=1, padx=4)
        ttk.Label(drive_frame, text="Hold Time (s)").grid(row=0, column=2, sticky="w")
        ttk.Entry(drive_frame, textvariable=self.drive_hold_var, width=10).grid(row=0, column=3, padx=4)
        ttk.Label(drive_frame, text="Max Torque").grid(row=0, column=4, sticky="w")
        ttk.Entry(drive_frame, textvariable=self.drive_torque_var, width=10).grid(row=0, column=5, padx=4)

        steer_frame = ttk.LabelFrame(motion, text="Steer Test Settings", padding=8)
        steer_frame.pack(side="left", fill="x", expand=True, padx=(6, 0))
        ttk.Label(steer_frame, text="Delta (rev)").grid(row=0, column=0, sticky="w")
        ttk.Entry(steer_frame, textvariable=self.steer_delta_var, width=10).grid(row=0, column=1, padx=4)
        ttk.Label(steer_frame, text="Hold Time (s)").grid(row=0, column=2, sticky="w")
        ttk.Entry(steer_frame, textvariable=self.steer_hold_var, width=10).grid(row=0, column=3, padx=4)
        ttk.Label(steer_frame, text="Max Torque").grid(row=0, column=4, sticky="w")
        ttk.Entry(steer_frame, textvariable=self.steer_torque_var, width=10).grid(row=0, column=5, padx=4)
        ttk.Label(steer_frame, text="Vel Limit").grid(row=0, column=6, sticky="w")
        ttk.Entry(steer_frame, textvariable=self.steer_vel_limit_var, width=10).grid(row=0, column=7, padx=4)

        ttk.Label(
            self.root,
            text="Wheels should be off the ground before running motion tests.",
            padding=(10, 2),
        ).pack(anchor="w")

        table_frame = ttk.Frame(self.root, padding=(10, 4))
        table_frame.pack(fill="both", expand=True)

        headers = [
            "ID",
            "Label",
            "State",
            "Mode",
            "Fault",
            "Voltage",
            "Position",
            "Velocity",
            "Torque",
            "Temp",
            "Query",
            "Test",
        ]
        for col, header in enumerate(headers):
            ttk.Label(table_frame, text=header).grid(row=0, column=col, padx=4, pady=2, sticky="w")

        for row_index, motor_id in enumerate(range(1, 9), start=1):
            vars_for_row = {
                "state": tk.StringVar(value="-"),
                "mode": tk.StringVar(value="-"),
                "fault": tk.StringVar(value="-"),
                "voltage": tk.StringVar(value="-"),
                "position": tk.StringVar(value="-"),
                "velocity": tk.StringVar(value="-"),
                "torque": tk.StringVar(value="-"),
                "temp": tk.StringVar(value="-"),
            }
            self.rows[motor_id] = vars_for_row

            ttk.Label(table_frame, text=str(motor_id)).grid(row=row_index, column=0, padx=4, pady=2, sticky="w")
            ttk.Label(table_frame, text=EXPECTED_MODULES[motor_id]).grid(row=row_index, column=1, padx=4, pady=2, sticky="w")
            ttk.Label(table_frame, textvariable=vars_for_row["state"]).grid(row=row_index, column=2, padx=4, pady=2, sticky="w")
            ttk.Label(table_frame, textvariable=vars_for_row["mode"]).grid(row=row_index, column=3, padx=4, pady=2, sticky="w")
            ttk.Label(table_frame, textvariable=vars_for_row["fault"]).grid(row=row_index, column=4, padx=4, pady=2, sticky="w")
            ttk.Label(table_frame, textvariable=vars_for_row["voltage"]).grid(row=row_index, column=5, padx=4, pady=2, sticky="w")
            ttk.Label(table_frame, textvariable=vars_for_row["position"]).grid(row=row_index, column=6, padx=4, pady=2, sticky="w")
            ttk.Label(table_frame, textvariable=vars_for_row["velocity"]).grid(row=row_index, column=7, padx=4, pady=2, sticky="w")
            ttk.Label(table_frame, textvariable=vars_for_row["torque"]).grid(row=row_index, column=8, padx=4, pady=2, sticky="w")
            ttk.Label(table_frame, textvariable=vars_for_row["temp"]).grid(row=row_index, column=9, padx=4, pady=2, sticky="w")
            ttk.Button(table_frame, text="Query", command=lambda mid=motor_id: self.queue_query_motor(mid)).grid(
                row=row_index,
                column=10,
                padx=4,
                pady=2,
            )
            ttk.Button(table_frame, text="Test", command=lambda mid=motor_id: self.queue_test_motor(mid)).grid(
                row=row_index,
                column=11,
                padx=4,
                pady=2,
            )

        log_frame = ttk.LabelFrame(self.root, text="Log", padding=8)
        log_frame.pack(fill="both", expand=True, padx=10, pady=(4, 10))
        self.log_text = tk.Text(log_frame, height=10, wrap="word")
        self.log_text.pack(fill="both", expand=True)
        self.log_text.configure(state="disabled")

        status = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        status.pack(fill="x")
        ttk.Label(status, textvariable=self.status_var).pack(anchor="w")

    def _append_log(self, message: str):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{time.strftime('%H:%M:%S')}  {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _get_timeout(self) -> float:
        return float(self.timeout_var.get().strip())

    def _get_motion_settings(self) -> dict[str, float]:
        return {
            "timeout": self._get_timeout(),
            "drive_rps": float(self.drive_rps_var.get().strip()),
            "drive_hold": float(self.drive_hold_var.get().strip()),
            "drive_torque": float(self.drive_torque_var.get().strip()),
            "steer_delta": float(self.steer_delta_var.get().strip()),
            "steer_hold": float(self.steer_hold_var.get().strip()),
            "steer_torque": float(self.steer_torque_var.get().strip()),
            "steer_vel_limit": float(self.steer_vel_limit_var.get().strip()),
        }

    def queue_query_all(self):
        self.status_var.set("Querying all motors...")
        self.task_queue.put(
            (
                "query_all",
                {
                    "ids": list(range(1, 9)),
                    "port": self.port_var.get().strip(),
                    "timeout": self._get_timeout(),
                },
            )
        )

    def queue_query_motor(self, motor_id: int):
        self.status_var.set(f"Querying motor {motor_id}...")
        self.task_queue.put(
            (
                "query_all",
                {
                    "ids": [motor_id],
                    "port": self.port_var.get().strip(),
                    "timeout": self._get_timeout(),
                },
            )
        )

    def queue_stop_all(self):
        self.status_var.set("Stopping all motors...")
        self.task_queue.put(
            (
                "stop_all",
                {
                    "ids": list(range(1, 9)),
                    "port": self.port_var.get().strip(),
                    "timeout": self._get_timeout(),
                },
            )
        )

    def queue_test_motor(self, motor_id: int):
        settings = self._get_motion_settings()
        self.status_var.set(f"Testing motor {motor_id}...")
        self.task_queue.put(
            (
                "test_motor",
                {
                    "motor_id": motor_id,
                    "port": self.port_var.get().strip(),
                    **settings,
                },
            )
        )

    def _worker_loop(self):
        while True:
            task_name, payload = self.task_queue.get()
            try:
                if task_name == "query_all":
                    port, results = asyncio.run(
                        query_ids_async(payload["port"], payload["ids"], payload["timeout"])
                    )
                    self.result_queue.put(("query_all", {"port": port, "results": results}))
                elif task_name == "stop_all":
                    port, statuses = asyncio.run(
                        stop_ids_async(payload["port"], payload["ids"], payload["timeout"])
                    )
                    self.result_queue.put(("stop_all", {"port": port, "statuses": statuses}))
                elif task_name == "test_motor":
                    port, before, after = asyncio.run(
                        test_motor_async(
                            port_arg=payload["port"],
                            motor_id=payload["motor_id"],
                            timeout_s=payload["timeout"],
                            drive_rps=payload["drive_rps"],
                            drive_hold_time=payload["drive_hold"],
                            drive_torque=payload["drive_torque"],
                            steer_delta=payload["steer_delta"],
                            steer_hold_time=payload["steer_hold"],
                            steer_torque=payload["steer_torque"],
                            steer_velocity_limit=payload["steer_vel_limit"],
                        )
                    )
                    self.result_queue.put(
                        (
                            "test_motor",
                            {
                                "port": port,
                                "motor_id": payload["motor_id"],
                                "before": before,
                                "after": after,
                            },
                        )
                    )
            except Exception as exc:
                self.result_queue.put(("error", {"message": repr(exc)}))
            finally:
                self.task_queue.task_done()

    def _poll_results(self):
        try:
            while True:
                event_name, payload = self.result_queue.get_nowait()
                if event_name == "query_all":
                    self._handle_query_results(payload["results"], payload["port"])
                elif event_name == "stop_all":
                    self._handle_stop_results(payload["statuses"], payload["port"])
                elif event_name == "test_motor":
                    self._handle_test_result(payload)
                elif event_name == "error":
                    self.status_var.set(f"Error: {payload['message']}")
                    self._append_log(f"ERROR {payload['message']}")
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self._poll_results)

    def _handle_query_results(self, results: list[QueryResult], port: str | None):
        for result in results:
            row = self.rows[result.motor_id]
            if not result.ok:
                row["state"].set("MISSING")
                row["mode"].set("-")
                row["fault"].set("-")
                row["voltage"].set("-")
                row["position"].set("-")
                row["velocity"].set("-")
                row["torque"].set("-")
                row["temp"].set(result.error or "-")
                continue

            mode = result.values.get("MODE")
            fault = result.values.get("FAULT")
            row["state"].set("FAULT" if fault not in (None, 0) else "OK")
            row["mode"].set(fmt(mode, 0))
            row["fault"].set(fmt(fault, 0))
            row["voltage"].set(fmt(result.values.get("VOLTAGE"), 2))
            row["position"].set(fmt(result.values.get("POSITION")))
            row["velocity"].set(fmt(result.values.get("VELOCITY")))
            row["torque"].set(fmt(result.values.get("TORQUE")))
            row["temp"].set(fmt(result.values.get("TEMPERATURE"), 1))

        self.status_var.set(f"Queried {len(results)} motor(s) on {port or 'moteus default'}")
        self._append_log(f"Queried {len(results)} motor(s) using {port or 'moteus default'}")

    def _handle_stop_results(self, statuses: dict[int, str], port: str | None):
        summary = ", ".join(f"{motor_id}:{status}" for motor_id, status in sorted(statuses.items()))
        self.status_var.set(f"Stop sent on {port or 'moteus default'}")
        self._append_log(f"Stop all -> {summary}")

    def _handle_test_result(self, payload: dict[str, Any]):
        motor_id = payload["motor_id"]
        before: QueryResult = payload["before"]
        after: QueryResult = payload["after"]
        port = payload["port"]

        self._handle_query_results([after], port)

        if not before.ok:
            self.status_var.set(f"Motor {motor_id} test failed before motion")
            self._append_log(f"Motor {motor_id} pre-query failed: {before.error}")
            return

        if not after.ok:
            self.status_var.set(f"Motor {motor_id} test failed after motion")
            self._append_log(f"Motor {motor_id} post-query failed: {after.error}")
            return

        before_pos = before.values.get("POSITION")
        after_pos = after.values.get("POSITION")
        before_vel = before.values.get("VELOCITY")
        after_vel = after.values.get("VELOCITY")

        self.status_var.set(f"Motor {motor_id} test complete")
        self._append_log(
            f"Motor {motor_id} {EXPECTED_MODULES[motor_id]} | "
            f"before_pos={fmt(before_pos)} before_vel={fmt(before_vel)} | "
            f"after_pos={fmt(after_pos)} after_vel={fmt(after_vel)}"
        )


def main():
    root = tk.Tk()
    style = ttk.Style()
    if "clam" in style.theme_names():
        style.theme_use("clam")
    app = MoteusGuiApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
