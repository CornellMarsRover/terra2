#!/usr/bin/env python3
"""
Small moteus scan intended for quick Jetson debugging.

This intentionally avoids the larger diagnostic script's formatting and motion
test code. It just reports which IDs respond and the most useful status fields.
"""

import argparse
import asyncio
import sys

import moteus


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


def value(values, register_name: str):
    register = getattr(moteus.Register, register_name, None)
    if register is None:
        return None
    return values.get(register, None)


async def main_async(args):
    transport = moteus.Fdcanusb(path=args.port)
    ids = parse_ids(args.ids)
    alive = []
    missing = []

    for motor_id in ids:
        controller = moteus.Controller(id=motor_id, transport=transport)
        try:
            result = await asyncio.wait_for(controller.query(), timeout=args.timeout)
            values = result.values
            alive.append(
                (
                    motor_id,
                    value(values, "MODE"),
                    value(values, "FAULT"),
                    value(values, "VOLTAGE"),
                    value(values, "POSITION"),
                    value(values, "VELOCITY"),
                )
            )
        except Exception as exc:
            missing.append((motor_id, repr(exc) if args.verbose else "no response"))

    print("Alive (id, mode, fault, voltage, position, velocity):")
    for row in alive:
        print(row)

    print("\nMissing/no response:")
    if missing:
        for row in missing:
            print(row)
    else:
        print("none")


def main():
    parser = argparse.ArgumentParser(description="Simple moteus CAN ID scan.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="fdcanusb device path.")
    parser.add_argument("--ids", default="1-32", help="IDs to scan, e.g. 1-32 or 3,5,6,7,8.")
    parser.add_argument("--timeout", type=float, default=0.10, help="Per-ID timeout in seconds.")
    parser.add_argument("--verbose", action="store_true", help="Print exception details for missing IDs.")
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc!r}", file=sys.stderr)
        print("Try checking the CAN adapter path with: ls -l /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/*", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
