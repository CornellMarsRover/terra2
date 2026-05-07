#!/usr/bin/env python3
"""
Moteus motor response diagnostic.

This is for checking controllers that respond over CAN but do not actually move.
It commands one motor at a time with a gentle velocity command and watches:
- position delta
- velocity
- q_current / d_current
- fault before and after

WHEELS OFF THE GROUND before using --test.
"""

import argparse
import asyncio
import math
import time
import sys

import moteus


LABELS = {
    1: "FL drive",
    2: "BL drive",
    3: "FR drive",
    4: "BR drive",
    5: "FL steer",
    6: "BL steer",
    7: "FR steer",
    8: "BR steer",
}


REGS = {
    name: getattr(moteus.Register, name, None)
    for name in [
        "MODE",
        "FAULT",
        "POSITION",
        "VELOCITY",
        "TORQUE",
        "VOLTAGE",
        "Q_CURRENT",
        "D_CURRENT",
        "TEMPERATURE",
        "MOTOR_TEMPERATURE",
    ]
}


def parse_ids(raw):
    ids = set()
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            ids.update(range(int(a), int(b) + 1))
        else:
            ids.add(int(part))
    return sorted(ids)


def get(result, name):
    reg = REGS.get(name)
    if reg is None:
        return None
    return result.values.get(reg)


def fmt(x, digits=3):
    if x is None:
        return "-"
    if isinstance(x, float):
        if math.isnan(x):
            return "nan"
        return f"{x:.{digits}f}"
    return str(x)


def make_qr():
    qr = moteus.QueryResolution()
    for field, resolution in [
        ("mode", moteus.INT8),
        ("fault", moteus.INT8),
        ("position", moteus.F32),
        ("velocity", moteus.F32),
        ("torque", moteus.F32),
        ("voltage", moteus.F32),
        ("q_current", moteus.F32),
        ("d_current", moteus.F32),
        ("temperature", moteus.F32),
        ("motor_temperature", moteus.F32),
    ]:
        if hasattr(qr, field):
            setattr(qr, field, resolution)
    return qr


async def stop(ctrl, timeout):
    try:
        await asyncio.wait_for(ctrl.set_stop(), timeout=timeout)
    except Exception:
        pass


async def query(ctrl, timeout):
    return await asyncio.wait_for(ctrl.query(), timeout=timeout)


async def test_one(motor_id, ctrl, args):
    label = LABELS.get(motor_id, "unknown")
    print(f"\n=== ID {motor_id} {label} ===")

    try:
        before = await query(ctrl, args.timeout)
    except Exception as exc:
        print(f"CAN/query: FAIL before test: {exc!r}")
        return

    before_fault = get(before, "FAULT")
    before_mode = get(before, "MODE")
    before_voltage = get(before, "VOLTAGE")
    before_pos = get(before, "POSITION")
    before_vel = get(before, "VELOCITY")
    before_q = get(before, "Q_CURRENT")
    before_d = get(before, "D_CURRENT")

    print(
        "Before: "
        f"mode={fmt(before_mode,0)} fault={fmt(before_fault,0)} "
        f"V={fmt(before_voltage,2)} pos={fmt(before_pos)} vel={fmt(before_vel)} "
        f"q={fmt(before_q)} d={fmt(before_d)}"
    )

    if not args.test:
        return

    if before_fault not in (None, 0):
        print("SKIP motion: controller already faulted.")
        return

    is_drive = 1 <= motor_id <= 4

    try:
        if is_drive:
            print(
                f"Command: drive velocity={args.drive_rps} rps, "
                f"torque={args.drive_torque}, duration={args.duration}s"
            )

            start = time.time()
            while time.time() - start < args.duration:
                await asyncio.wait_for(
                    ctrl.set_position(
                        position=math.nan,
                        velocity=args.drive_rps,
                        maximum_torque=args.drive_torque,
                        accel_limit=args.accel_limit,
                        watchdog_timeout=0.25,
                    ),
                    timeout=args.timeout,
                )
                await asyncio.sleep(0.05)
        else:
            target = (before_pos or 0.0) + args.steer_delta
            print(
                f"Command: steer target_delta={args.steer_delta} rev, "
                f"target={fmt(target)}, torque={args.steer_torque}"
            )
            await asyncio.wait_for(
                ctrl.set_position(
                    position=target,
                    velocity_limit=args.steer_velocity_limit,
                    maximum_torque=args.steer_torque,
                    watchdog_timeout=max(0.5, args.duration + 0.25),
                ),
                timeout=args.timeout,
            )
            await asyncio.sleep(args.duration)

        after = await query(ctrl, args.timeout)
        await stop(ctrl, args.timeout)

    except Exception as exc:
        print(f"Command/query after: FAIL: {exc!r}")
        await stop(ctrl, args.timeout)
        return

    after_fault = get(after, "FAULT")
    after_mode = get(after, "MODE")
    after_voltage = get(after, "VOLTAGE")
    after_pos = get(after, "POSITION")
    after_vel = get(after, "VELOCITY")
    after_q = get(after, "Q_CURRENT")
    after_d = get(after, "D_CURRENT")

    delta_pos = None
    delta_q = None
    delta_d = None

    if before_pos is not None and after_pos is not None:
        delta_pos = after_pos - before_pos
    if before_q is not None and after_q is not None:
        delta_q = after_q - before_q
    if before_d is not None and after_d is not None:
        delta_d = after_d - before_d

    print(
        "After:  "
        f"mode={fmt(after_mode,0)} fault={fmt(after_fault,0)} "
        f"V={fmt(after_voltage,2)} pos={fmt(after_pos)} vel={fmt(after_vel)} "
        f"q={fmt(after_q)} d={fmt(after_d)}"
    )

    print(
        "Delta:  "
        f"pos={fmt(delta_pos)} q={fmt(delta_q)} d={fmt(delta_d)}"
    )

    moved = delta_pos is not None and abs(delta_pos) >= args.min_delta
    current_changed = (
        (delta_q is not None and abs(delta_q) >= args.min_current_delta)
        or (delta_d is not None and abs(delta_d) >= args.min_current_delta)
    )
    faulted = after_fault not in (None, 0)

    print("Interpretation:")

    if faulted:
        print(f"  FAIL: controller faulted after command, fault={after_fault}.")
        print("  Suspect motor phase wiring, encoder/config, controller config, or mechanical bind.")
    elif moved:
        print("  PASS: position changed. Motor/controller/mechanics responded.")
    elif current_changed:
        print("  WARN: current changed but position did not.")
        print("  Suspect mechanical bind, brake, disconnected output, or motor unable to move.")
    else:
        print("  FAIL: no position change and no meaningful current change.")
        print("  Suspect command not accepted, controller config, motor disconnected, or wrong wiring.")


async def main_async(args):
    ids = parse_ids(args.ids)
    qr = make_qr()
    transport = moteus.Fdcanusb(args.port)

    print(f"Using fdcanusb: {args.port}")
    print(f"IDs: {ids}")

    if args.test:
        print("\nMOTION TEST ENABLED. WHEELS MUST BE OFF THE GROUND.\n")
    else:
        print("\nScan only. Add --test to command motion.\n")

    for motor_id in ids:
        ctrl = moteus.Controller(id=motor_id, transport=transport, query_resolution=qr)
        await test_one(motor_id, ctrl, args)
        await asyncio.sleep(0.15)


def main():
    parser = argparse.ArgumentParser(description="Check moteus motor response beyond CAN communication.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--ids", default="1-8")
    parser.add_argument("--timeout", type=float, default=1.0)

    parser.add_argument("--test", action="store_true", help="Actually command gentle motion.")
    parser.add_argument("--duration", type=float, default=0.5)

    parser.add_argument("--drive-rps", type=float, default=0.2)
    parser.add_argument("--drive-torque", type=float, default=0.3)
    parser.add_argument("--accel-limit", type=float, default=2.0)

    parser.add_argument("--steer-delta", type=float, default=0.25)
    parser.add_argument("--steer-torque", type=float, default=0.5)
    parser.add_argument("--steer-velocity-limit", type=float, default=1.0)

    parser.add_argument("--min-delta", type=float, default=0.02)
    parser.add_argument("--min-current-delta", type=float, default=0.25)

    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc!r}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
