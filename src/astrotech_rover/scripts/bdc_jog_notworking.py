#!/usr/bin/env python3
"""Command one Astrotech BDC motor by board CAN ID and motor port.

Examples:

    python3 src/astrotech_rover/scripts/bdc_jog.py --can-id 20 --motor-id 0 --dir fwd --seconds 1
    python3 src/astrotech_rover/scripts/bdc_jog.py --can-id 20 --motor-id 0 --stop
    python3 src/astrotech_rover/scripts/bdc_jog.py --can-id 18 --motor-id 3 --clear-faults
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

from CMR_CANFD import BDCController, FdCanInterface  # noqa: E402

DEFAULT_PORT = "/dev/serial/by-id/usb-mjbots_fdcanusb_8249C85D-if00"
DEFAULT_BITRATE = 1_000_000
MAX_SECONDS = 127
MAX_MS = 127


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"fdcanusb serial device (default: {DEFAULT_PORT})")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                        help=f"CAN bitrate (default: {DEFAULT_BITRATE})")
    parser.add_argument("--can-id", type=int, required=True,
                        help="BDC board CAN ID, e.g. 18, 20, or 21")
    parser.add_argument("--motor-id", type=int, required=True,
                        help="BDC motor port on that board, 0..5")
    parser.add_argument("--dir", choices=("fwd", "rev"), default="fwd",
                        help="direction for a move command")
    parser.add_argument("--seconds", type=int, default=0,
                        help=f"move duration in seconds, 0..{MAX_SECONDS}")
    parser.add_argument("--ms", type=int, default=0,
                        help=f"move duration in milliseconds, 0..{MAX_MS}")
    parser.add_argument("--stop", action="store_true",
                        help="send stop_motor instead of a move")
    parser.add_argument("--clear-faults", action="store_true",
                        help="send clear_faults before any move/stop")
    parser.add_argument("--listen-s", type=float, default=0.25,
                        help="seconds to keep the port open after sending")
    return parser.parse_args()


def _validate(args: argparse.Namespace) -> None:
    if not 0 <= args.motor_id <= 5:
        raise SystemExit("--motor-id must be in range 0..5")
    if args.seconds and args.ms:
        raise SystemExit("use only one of --seconds or --ms")
    if args.seconds < 0 or args.seconds > MAX_SECONDS:
        raise SystemExit(f"--seconds must be in range 0..{MAX_SECONDS}")
    if args.ms < 0 or args.ms > MAX_MS:
        raise SystemExit(f"--ms must be in range 0..{MAX_MS}")
    if not args.stop and not args.clear_faults and args.seconds == 0 and args.ms == 0:
        raise SystemExit("nothing to do: pass --seconds N, --ms N, --stop, or --clear-faults")


async def _configure_bus(fd: FdCanInterface, bitrate: int) -> None:
    for cmd in (
        "can off",
        f"conf set can.bitrate {bitrate}",
        "conf set can.fdcan_frame off",
        "conf set can.bitrate_switch on",
        "conf set can.termination on",
        "can on",
    ):
        resp = await fd._send_command(cmd)
        print(f"  > {cmd}  ->  {resp}")


async def main_async(args: argparse.Namespace) -> int:
    _validate(args)

    fd = FdCanInterface(port=args.port, baud=115200)
    print(f"opening {args.port}...")
    await fd.open()
    try:
        print(f"configuring bus @ {args.bitrate} bps...")
        await _configure_bus(fd, args.bitrate)

        bdc = BDCController(can=fd, motor_id=args.motor_id, can_id=args.can_id)
        print(f"target: can_id={args.can_id} motor_id={args.motor_id}")

        if args.clear_faults:
            print("sending clear_faults")
            await bdc.clear_faults()
            await asyncio.sleep(0.05)

        if args.stop:
            print("sending stop_motor")
            await bdc.stop_motor()
        elif args.ms:
            print(f"sending move_motor_{args.dir}_ms({args.ms})")
            if args.dir == "rev":
                await bdc.move_motor_reverse_ms(args.ms)
            else:
                await bdc.move_motor_forward_ms(args.ms)
        elif args.seconds:
            print(f"sending move_motor_{args.dir}({args.seconds})")
            if args.dir == "rev":
                await bdc.move_motor_reverse(args.seconds)
            else:
                await bdc.move_motor_forward(args.seconds)

        if args.listen_s > 0:
            print(f"holding port open for {args.listen_s}s...")
            await asyncio.sleep(args.listen_s)
    finally:
        await fd.close()
    print("closed.")
    return 0


def main() -> int:
    try:
        return asyncio.run(main_async(_parse_args()))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
