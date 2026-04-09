#!/usr/bin/env python3
"""
Moteus drivetrain diagnostic for the CMR rover.

By default this script only queries controllers. It does not command motion unless
--motion-test is passed.
"""

import argparse
import asyncio
import csv
import glob
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Any

import moteus


DEFAULT_PORT = "auto"

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


def make_query_resolution() -> moteus.QueryResolution:
    qr = moteus.QueryResolution()

    # Keep everything compact enough for a scan, but include the fields that
    # usually separate "software mapping" from "controller/power/fault" issues.
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


def parse_ids(raw: str) -> list[int]:
    ids: set[int] = set()
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = part.split("-", 1)
            ids.update(range(int(start), int(end) + 1))
        else:
            ids.add(int(part))
    return sorted(ids)


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


def status_label(mode: Any, fault: Any) -> str:
    if fault not in (None, 0):
        return "FAULT"
    if mode is None:
        return "UNKNOWN"
    return "OK"


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


async def query_all(controllers, timeout_s: float) -> list[QueryResult]:
    tasks = [
        query_one(ctrl, motor_id, label, timeout_s)
        for motor_id, (label, ctrl) in controllers.items()
    ]
    return await asyncio.gather(*tasks)


def print_table(results: list[QueryResult], min_voltage: float):
    headers = [
        "id",
        "label",
        "status",
        "mode",
        "fault",
        "volts",
        "pos",
        "vel",
        "torque",
        "q_cur",
        "d_cur",
        "temp",
        "motor_temp",
        "notes",
    ]
    rows = []

    for result in sorted(results, key=lambda x: x.motor_id):
        if not result.ok:
            rows.append([
                result.motor_id,
                result.label,
                "MISSING",
                "-",
                "-",
                "-",
                "-",
                "-",
                "-",
                "-",
                "-",
                "-",
                "-",
                result.error,
            ])
            continue

        mode = result.values.get("MODE")
        fault = result.values.get("FAULT")
        volts = result.values.get("VOLTAGE")
        notes = []
        if fault not in (None, 0):
            notes.append("fault nonzero")
        if volts is not None and volts < min_voltage:
            notes.append(f"voltage < {min_voltage:g}")

        rows.append([
            result.motor_id,
            result.label,
            status_label(mode, fault),
            fmt(mode, 0),
            fmt(fault, 0),
            fmt(volts, 2),
            fmt(result.values.get("POSITION")),
            fmt(result.values.get("VELOCITY")),
            fmt(result.values.get("TORQUE")),
            fmt(result.values.get("Q_CURRENT")),
            fmt(result.values.get("D_CURRENT")),
            fmt(result.values.get("TEMPERATURE"), 1),
            fmt(result.values.get("MOTOR_TEMPERATURE"), 1),
            "; ".join(notes) if notes else "",
        ])

    widths = [
        max(len(str(row[i])) for row in [headers] + rows)
        for i in range(len(headers))
    ]
    print("  ".join(header.ljust(widths[i]) for i, header in enumerate(headers)))
    print("  ".join("-" * width for width in widths))
    for row in rows:
        print("  ".join(str(value).ljust(widths[i]) for i, value in enumerate(row)))


async def clear_faults(controllers, timeout_s: float):
    print("\nClearing faults with set_stop(clear_faults=True)...")
    tasks = []
    for _motor_id, (_label, ctrl) in controllers.items():
        tasks.append(asyncio.wait_for(ctrl.set_stop(clear_faults=True), timeout=timeout_s))
    results = await asyncio.gather(*tasks, return_exceptions=True)
    for (motor_id, (label, _ctrl)), result in zip(controllers.items(), results):
        if isinstance(result, Exception):
            print(f"  id {motor_id:>2} {label:<8}: failed: {result!r}")
        else:
            print(f"  id {motor_id:>2} {label:<8}: stop/clear sent")


async def motion_test(controllers, ids: list[int], timeout_s: float, drive_rps: float, steer_delta: float):
    print("\nMOTION TEST ENABLED. Wheels should be off the ground.")
    print("Sending tiny commands one motor at a time, then stopping each motor.")

    for motor_id in ids:
        label, ctrl = controllers[motor_id]
        is_drive = 1 <= motor_id <= 4
        print(f"\nTesting id {motor_id} {label}...")
        try:
            before = await asyncio.wait_for(ctrl.query(), timeout=timeout_s)
            before_pos = read_value(before, "POSITION")

            if is_drive:
                await asyncio.wait_for(
                    ctrl.set_position(
                        position=math.nan,
                        velocity=drive_rps,
                        maximum_torque=1.0,
                        accel_limit=5.0,
                        watchdog_timeout=0.25,
                    ),
                    timeout=timeout_s,
                )
                await asyncio.sleep(0.35)
            else:
                target = (before_pos or 0.0) + steer_delta
                await asyncio.wait_for(
                    ctrl.set_position(
                        position=target,
                        velocity_limit=1.0,
                        maximum_torque=1.0,
                        watchdog_timeout=0.5,
                    ),
                    timeout=timeout_s,
                )
                await asyncio.sleep(0.50)

            after = await asyncio.wait_for(ctrl.query(), timeout=timeout_s)
            await asyncio.wait_for(ctrl.set_stop(), timeout=timeout_s)

            after_pos = read_value(after, "POSITION")
            after_vel = read_value(after, "VELOCITY")
            print(
                f"  before_pos={fmt(before_pos)} after_pos={fmt(after_pos)} "
                f"after_vel={fmt(after_vel)}"
            )
        except Exception as exc:
            print(f"  FAILED: {exc!r}")
            try:
                await asyncio.wait_for(ctrl.set_stop(), timeout=timeout_s)
            except Exception:
                pass


def summarize(results: list[QueryResult], expected_ids: list[int], min_voltage: float):
    missing = [x for x in results if not x.ok]
    faulted = [
        x for x in results
        if x.ok and x.values.get("FAULT") not in (None, 0)
    ]
    low_voltage = [
        x for x in results
        if x.ok and x.values.get("VOLTAGE") is not None and x.values["VOLTAGE"] < min_voltage
    ]

    print("\nSummary:")
    print(f"  expected IDs: {expected_ids}")
    print(f"  responding: {[x.motor_id for x in results if x.ok]}")
    print(f"  missing: {[x.motor_id for x in missing] or 'none'}")
    print(f"  faulted: {[(x.motor_id, x.values.get('FAULT')) for x in faulted] or 'none'}")
    print(f"  low voltage: {[(x.motor_id, fmt(x.values.get('VOLTAGE'), 2)) for x in low_voltage] or 'none'}")

    if 3 in [x.motor_id for x in missing]:
        print("  note: FR drive is expected to be moteus id 3 and did not respond.")
    steer_missing = [x.motor_id for x in missing if x.motor_id in (5, 6, 7, 8)]
    if steer_missing:
        print(f"  note: steer IDs missing: {steer_missing}. Check steer bank power/CAN/IDs.")


def write_csv(path: str, results: list[QueryResult]):
    fieldnames = [
        "timestamp",
        "id",
        "label",
        "ok",
        "error",
        *REGISTER_NAMES.keys(),
    ]
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        now = time.time()
        for result in results:
            row = {
                "timestamp": now,
                "id": result.motor_id,
                "label": result.label,
                "ok": result.ok,
                "error": result.error or "",
            }
            row.update(result.values)
            writer.writerow(row)
    print(f"\nWrote CSV: {path}")


async def main_async(args):
    ids = parse_ids(args.ids)
    qr = make_query_resolution()

    transport = None
    port = resolve_port(args.port)
    if port:
        transport = moteus.Fdcanusb(port)

    controllers = {
        motor_id: (
            EXPECTED_MODULES.get(motor_id, "unknown"),
            make_controller(motor_id, transport, qr),
        )
        for motor_id in ids
    }

    if args.port == "auto":
        found_ports = find_can_ports()
        print(f"Detected CAN USB candidates: {found_ports or 'none'}")
    print(f"Using CAN transport: {port or 'moteus default'}")
    print(f"Querying IDs: {ids}")

    if args.clear_faults:
        await clear_faults(controllers, args.timeout)
        await asyncio.sleep(0.2)

    all_results: list[QueryResult] = []
    for sample in range(args.samples):
        if args.samples > 1:
            print(f"\nSample {sample + 1}/{args.samples}")
        results = await query_all(controllers, args.timeout)
        print_table(results, args.min_voltage)
        all_results = results
        if sample + 1 < args.samples:
            await asyncio.sleep(args.interval)

    summarize(all_results, ids, args.min_voltage)

    if args.csv:
        write_csv(args.csv, all_results)

    if args.motion_test:
        await motion_test(controllers, ids, args.timeout, args.drive_rps, args.steer_delta)
        print("\nFinal status after motion test:")
        final_results = await query_all(controllers, args.timeout)
        print_table(final_results, args.min_voltage)
        summarize(final_results, ids, args.min_voltage)


def main():
    parser = argparse.ArgumentParser(
        description="Query moteus drivetrain IDs, voltages, faults, positions, currents, and temperatures."
    )
    parser.add_argument(
        "--ids",
        default="1-8",
        help="IDs to query, like '1-8' or '1,2,3,5,6,7,8'. Default: 1-8",
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=(
            "fdcanusb port, 'auto', or '' for moteus default. "
            f"Default: {DEFAULT_PORT}"
        ),
    )
    parser.add_argument("--timeout", type=float, default=0.20, help="Per-command timeout seconds.")
    parser.add_argument("--samples", type=int, default=1, help="Number of query samples.")
    parser.add_argument("--interval", type=float, default=0.50, help="Seconds between samples.")
    parser.add_argument("--min-voltage", type=float, default=18.0, help="Flag voltage below this value.")
    parser.add_argument("--csv", default="", help="Optional CSV output path.")
    parser.add_argument("--clear-faults", action="store_true", help="Send set_stop(clear_faults=True) before querying.")
    parser.add_argument(
        "--motion-test",
        action="store_true",
        help="Actually command tiny one-at-a-time motion tests. Wheels off ground.",
    )
    parser.add_argument("--drive-rps", type=float, default=1.0, help="Drive test velocity in motor rev/s.")
    parser.add_argument("--steer-delta", type=float, default=0.05, help="Steer test position delta in motor rev.")
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        print(
            "Plug the fdcanusb into this computer, or run this on the Jetson, "
            "or pass the correct device with --port /dev/ttyACM1.",
            file=sys.stderr,
        )
        return 2
    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
