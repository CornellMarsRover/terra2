#!/usr/bin/env python3

import argparse
import asyncio
import math
import time

import moteus


JOINT_MOTOR_IDS = {
    "base_joint": 9,
    "shoulder_joint": 10,
    "elbow_joint": 11,
    "wrist_rotate": 12,
    "wrist_twist": 13,
    "wrist_rotate_two": 14,
}


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Read one arm moteus position, command a tiny relative move, then stop. "
            "This bypasses ROS trajectories, MoveIt, Servo, and IK."
        ),
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--joint", choices=JOINT_MOTOR_IDS.keys(), default="wrist_twist")
    parser.add_argument("--motor-id", type=int, default=None)
    parser.add_argument("--delta-rev", type=float, default=0.01)
    parser.add_argument("--duration", type=float, default=0.35)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--velocity-limit", type=float, default=0.25)
    parser.add_argument("--accel-limit", type=float, default=1.0)
    parser.add_argument("--max-torque", type=float, default=0.2)
    return parser


def register_value(result, register):
    return result.values.get(register)


async def query_position(ctrl, timeout):
    result = await asyncio.wait_for(ctrl.query(), timeout=timeout)
    position = register_value(result, moteus.Register.POSITION)
    fault = register_value(result, moteus.Register.FAULT)
    voltage = register_value(result, moteus.Register.VOLTAGE)
    if position is None or math.isnan(position):
        raise RuntimeError("moteus query did not return a valid position")
    return position, fault, voltage


async def command_nudge(ctrl, target, args):
    start = time.time()
    watchdog_timeout = max(0.15, min(0.5, args.duration + 0.15))
    while time.time() - start < args.duration:
        await asyncio.wait_for(
            ctrl.set_position(
                position=target,
                velocity_limit=args.velocity_limit,
                accel_limit=args.accel_limit,
                maximum_torque=args.max_torque,
                watchdog_timeout=watchdog_timeout,
            ),
            timeout=args.timeout,
        )
        await asyncio.sleep(0.05)


async def main_async(args):
    motor_id = args.motor_id if args.motor_id is not None else JOINT_MOTOR_IDS[args.joint]
    transport = moteus.Fdcanusb(args.port)
    ctrl = moteus.Controller(id=motor_id, transport=transport)

    before, fault, voltage = await query_position(ctrl, args.timeout)
    if fault not in (None, 0):
        raise RuntimeError(f"motor {motor_id} is faulted before motion: fault={fault}")

    target = before + args.delta_rev
    print(
        f"{args.joint} motor {motor_id}: "
        f"V={voltage:.2f} current={before:.4f} rev target={target:.4f} rev"
    )

    try:
        await command_nudge(ctrl, target, args)
        after, after_fault, _ = await query_position(ctrl, args.timeout)
        print(
            f"{args.joint} motor {motor_id}: "
            f"after={after:.4f} rev actual_delta={after - before:.4f} rev "
            f"fault={after_fault}"
        )
    finally:
        await asyncio.wait_for(ctrl.set_stop(), timeout=args.timeout)
        print(f"{args.joint} motor {motor_id}: sent set_stop()")


def main():
    args = build_parser().parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
