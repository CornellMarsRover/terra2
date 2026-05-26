#!/usr/bin/env python3
"""Minimal servo move using CMR_CANFD. Bus must be 1 Mbps.

This is a near-verbatim adaptation of the bench-validated script the user
ran successfully against the rig (the version pasted into chat on
2026-05-08). The only difference is that PORT / BOARD_CAN_ID / SERVO_PORT
/ target angle are CLI args instead of module-level constants, so we
don't have to edit the file for every nudge.

Imports the vendored ``CMR_CANFD`` library directly (same as the user's
working script). That requires ``pyserial-asyncio`` (``pip install
pyserial-asyncio``); see ``src/astrotech_rover/third_party/astrotech_canfd/API_NOTES.md``.

Use this when the more elaborate ``mixing_servo_jog.py`` doesn't seem to
move the rig: this version is the absolute simplest path that has been
demonstrated to work, with no background reader thread, no auto-stop,
no pyserial layer of its own.

Example:

    python3 src/astrotech_rover/scripts/mixing_servo_min.py --pos 5
    python3 src/astrotech_rover/scripts/mixing_servo_min.py --pos 10
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), "..", "third_party",
    "astrotech_canfd", "servo_bdc_control_ms", "test",
))

from CMR_CANFD import FdCanInterface, ServoController  # noqa: E402

DEFAULT_PORT = "/dev/serial/by-id/usb-mjbots_fdcanusb_8249C85D-if00"
DEFAULT_BOARD_CAN_ID = 26
DEFAULT_SERVO_PORT = 15
DEFAULT_BITRATE = 1_000_000


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--pos", type=int, required=True,
                   help="absolute target angle in degrees (0..4095)")
    p.add_argument("--port", default=DEFAULT_PORT,
                   help=f"serial device (default: {DEFAULT_PORT})")
    p.add_argument("--can-id", type=int, default=DEFAULT_BOARD_CAN_ID,
                   help=f"servo board CAN id (default: {DEFAULT_BOARD_CAN_ID})")
    p.add_argument("--servo-id", type=int, default=DEFAULT_SERVO_PORT,
                   help=f"servo slot on the board, 0..15 "
                        f"(default: {DEFAULT_SERVO_PORT})")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                   help=f"CAN bus bitrate (default: {DEFAULT_BITRATE})")
    p.add_argument("--no-clear-faults", action="store_true",
                   help="opt out of setting clear_faults=1 on the goto "
                        "frame. Default is on -- without it the rig has "
                        "been observed to ACK every goto on the bus but "
                        "ignore them mechanically (2026-05-08 bench "
                        "bring-up). Only pass this flag for diagnostic "
                        "experiments.")
    p.add_argument("--also-set-home", type=int, default=None, metavar="N",
                   help="before the goto, send set_home(N) (control_mode=3, "
                        "reset_home=1). Persists board-side. Try this if "
                        "the servo firmware rejects positions until a home "
                        "has been registered for this session.")
    p.add_argument("--hold-s", type=float, default=None, metavar="S",
                   help="seconds to hold the port open AFTER sending the "
                        "goto frame before closing. Default: hold until "
                        "Ctrl-C (closing too early can race the mechanical "
                        "move on this firmware -- the bus ACKs the frame "
                        "but the chamber stops mid-travel when the port "
                        "closes). Pass 0 for the original immediate-close "
                        "behaviour, or e.g. --hold-s 3 for a fixed wait.")
    return p.parse_args()


async def main_async(args: argparse.Namespace) -> int:
    fd = FdCanInterface(port=args.port, baud=115200)
    print(f"opening {args.port}...")
    await fd.open()

    print(f"running configure_bus @ {args.bitrate} bps...")
    for cmd in (
        "can off",
        f"conf set can.bitrate {args.bitrate}",
        "conf set can.fdcan_frame off",
        "conf set can.bitrate_switch on",
        "conf set can.termination on",
        "can on",
    ):
        resp = await fd._send_command(cmd)
        print(f"  > {cmd}  ->  {resp}")

    servo = ServoController(can=fd, servo_id=args.servo_id, can_id=args.can_id)

    if args.also_set_home is not None:
        print(f"set_home({args.also_set_home}) on (can_id={args.can_id}, "
              f"servo_id={args.servo_id})")
        await servo.set_home(args.also_set_home)
        await asyncio.sleep(0.05)

    # Default: clear_faults=1. ServoController has no clear_faults()
    # helper, so build the frame by hand using the same internal
    # _make_control_frame. control_mode=0 / control_data=args.pos plus
    # clear_faults=1 (byte3 bit 7) is the same goto we'd otherwise
    # send via servo.go_to_position(), with the fault-clear bit
    # asserted. See --no-clear-faults for opt-out.
    if args.no_clear_faults:
        print(f"go_to_position({args.pos}) on (can_id={args.can_id}, "
              f"servo_id={args.servo_id})  [clear_faults disabled]")
        await servo.go_to_position(args.pos)
    else:
        frame = servo._make_control_frame(
            control_mode=0, control_data=args.pos, clear_faults=1
        )
        print(f"go_to_position({args.pos}) +clear_faults=1 on "
              f"(can_id={args.can_id}, servo_id={args.servo_id})")
        await fd.write_frame(
            std_id=servo.can_id, data_hex=frame.hex(), flags="FB"
        )

    # Match the user's working script exactly: no stop, no auto-stop, no
    # follow-up commands. The port stays open until the user signals (or
    # --hold-s N elapses) so the chamber actually finishes its mechanical
    # move -- closing too early can cancel an in-flight goto on this
    # firmware (bus ACKs but the rig stops mid-travel).
    try:
        if args.hold_s is None:
            print("frame sent. holding port open -- press Ctrl-C when the "
                  "chamber has reached the target.")
            try:
                while True:
                    await asyncio.sleep(3600)
            except (asyncio.CancelledError, KeyboardInterrupt):
                pass
        elif args.hold_s > 0:
            print(f"frame sent. holding port for {args.hold_s}s...")
            await asyncio.sleep(args.hold_s)
    finally:
        try:
            await fd.close()
        except Exception as exc:  # pragma: no cover - hardware path
            print(f"warning: close raised {exc}", file=sys.stderr)
    print("closed.")
    return 0


def main() -> int:
    args = _parse_args()
    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
