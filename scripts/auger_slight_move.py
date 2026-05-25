#!/usr/bin/env python3
"""Conservative one-shot moteus probe for the Astrotech auger stack.

Two controllers on the same fdcanusb transport:

    id=15  lead_screw  (vertical motion)
    id=16  auger       (rotational drill)

This script is the safer cousin of ``docs/reference/auger_keys_test_harness.py``:

* No ``keyboard`` library, no sudo, no infinite loop.
* Pings both controllers first; refuses to move anything if either is missing.
* Clamps user-supplied velocity / torque to hardware-safe envelopes, with the
  defaults intentionally well below the reference harness numbers.
* The motor that is *not* being driven is actively held at ``velocity=0``
  with a small torque, so the lead screw cannot free-wheel under gravity.
* ``try/finally`` always issues ``make_stop()`` on both controllers, even on
  exception or Ctrl-C.

This is for hardware bring-up only. The real Phase 2b moteus ROS driver will
live elsewhere; see ``TODO(phase-2b-moteus)`` in
``src/urc_mock_rover/urc_mock_rover/drivers/auger.py``.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
import time

import moteus

LEAD_SCREW_ID = 15
AUGER_ID = 16

LOOP_HZ = 50.0  # match the reference harness cadence so the watchdog stays fed.
PING_TIMEOUT_S = 1.0  # transport.cycle() has no internal timeout; wrap it.

# Safe envelopes for a "slight movement" first probe. The reference harness
# runs at 10 / -50 / 100 rev/s with 1-2 Nm max torque; we cap an order of
# magnitude under that so a typo can't fling the rig.
SAFE_MAX_VELOCITY_REV_S = 2.0
SAFE_MAX_TORQUE_NM = 1.0
SAFE_MAX_DURATION_S = 5.0

DEFAULT_MOTOR = "auger"
DEFAULT_VELOCITY_REV_S = 0.5
DEFAULT_DURATION_S = 1.0
DEFAULT_MAX_TORQUE_NM = 0.3
HOLD_TORQUE_NM = 0.3  # torque used to hold the not-driven motor at zero.


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--motor",
        choices=("auger", "lead_screw"),
        default=DEFAULT_MOTOR,
        help="Which motor to drive. The other one is actively held at zero.",
    )
    p.add_argument(
        "--velocity",
        type=float,
        default=DEFAULT_VELOCITY_REV_S,
        help=f"Magnitude in rev/s. Capped at {SAFE_MAX_VELOCITY_REV_S}.",
    )
    p.add_argument(
        "--reverse",
        action="store_true",
        help="Negate the velocity sign.",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help=f"Run duration in seconds. Capped at {SAFE_MAX_DURATION_S}.",
    )
    p.add_argument(
        "--max-torque",
        type=float,
        default=DEFAULT_MAX_TORQUE_NM,
        help=f"Per-motor max torque in Nm. Capped at {SAFE_MAX_TORQUE_NM}.",
    )
    p.add_argument(
        "--ping-only",
        action="store_true",
        help="Verify both controllers are on the bus and exit without moving.",
    )
    args = p.parse_args()

    if args.velocity < 0:
        p.error("--velocity must be non-negative; use --reverse for direction.")
    if args.velocity > SAFE_MAX_VELOCITY_REV_S:
        p.error(
            f"--velocity {args.velocity} > safe cap "
            f"{SAFE_MAX_VELOCITY_REV_S} rev/s; raise the cap deliberately."
        )
    if args.max_torque <= 0 or args.max_torque > SAFE_MAX_TORQUE_NM:
        p.error(
            f"--max-torque must be in (0, {SAFE_MAX_TORQUE_NM}] Nm "
            f"(got {args.max_torque})."
        )
    if args.duration <= 0 or args.duration > SAFE_MAX_DURATION_S:
        p.error(
            f"--duration must be in (0, {SAFE_MAX_DURATION_S}] s "
            f"(got {args.duration})."
        )
    return args


def _fmt(result: moteus.Result | None) -> str:
    if result is None:
        return "no reply"
    v = result.values
    pos = v.get(moteus.Register.POSITION, float("nan"))
    vel = v.get(moteus.Register.VELOCITY, float("nan"))
    trq = v.get(moteus.Register.TORQUE, float("nan"))
    temp = v.get(moteus.Register.TEMPERATURE, float("nan"))
    mode = v.get(moteus.Register.MODE, -1)
    fault = v.get(moteus.Register.FAULT, -1)
    return (
        f"mode={int(mode):>2} fault={int(fault):>2} "
        f"pos={pos:+7.3f} rev  vel={vel:+6.2f} rev/s  "
        f"trq={trq:+5.2f} Nm  temp={temp:5.1f} C"
    )


async def _ping(transport: moteus.Transport, c: moteus.Controller, label: str) -> bool:
    try:
        result = await asyncio.wait_for(
            transport.cycle([c.make_query()]), timeout=PING_TIMEOUT_S
        )
    except asyncio.TimeoutError:
        print(
            f"  {label} (id={c.id}): no reply within {PING_TIMEOUT_S:.1f} s",
            file=sys.stderr,
        )
        return False
    except Exception as exc:  # pragma: no cover - hardware path
        print(f"  {label} (id={c.id}): transport error: {exc}", file=sys.stderr)
        return False
    if not result:
        print(f"  {label} (id={c.id}): no reply on the bus", file=sys.stderr)
        return False
    print(f"  {label} (id={c.id}): {_fmt(result[0])}")
    return True


async def main_async(args: argparse.Namespace) -> int:
    transport = moteus.get_singleton_transport()
    lead_screw = moteus.Controller(id=LEAD_SCREW_ID)
    auger = moteus.Controller(id=AUGER_ID)

    print(f"transport: {type(transport).__name__}")
    print("pinging controllers...")
    ok_ls = await _ping(transport, lead_screw, "lead_screw")
    ok_a = await _ping(transport, auger, "auger     ")
    if not (ok_ls and ok_a):
        print("\nrefusing to move: one or both controllers did not respond.",
              file=sys.stderr)
        return 2

    print("\nclearing faults via make_stop on both controllers...")
    await transport.cycle([lead_screw.make_stop(), auger.make_stop()])

    if args.ping_only:
        print("ping-only: done.")
        return 0

    target_velocity = -args.velocity if args.reverse else args.velocity
    is_auger = args.motor == "auger"
    driven_label = "auger" if is_auger else "lead_screw"
    print(
        f"\nrunning {driven_label} at {target_velocity:+.2f} rev/s, "
        f"max_torque={args.max_torque} Nm, duration={args.duration:.2f} s "
        f"(loop @ {LOOP_HZ:.0f} Hz). Ctrl-C aborts cleanly."
    )

    period = 1.0 / LOOP_HZ
    n_cycles = max(1, int(round(args.duration * LOOP_HZ)))
    started_monotonic = time.monotonic()
    try:
        for i in range(n_cycles):
            lead_cmd = lead_screw.make_position(
                position=math.nan,
                velocity=(target_velocity if not is_auger else 0.0),
                maximum_torque=(args.max_torque if not is_auger else HOLD_TORQUE_NM),
                query=True,
            )
            auger_cmd = auger.make_position(
                position=math.nan,
                velocity=(target_velocity if is_auger else 0.0),
                maximum_torque=(args.max_torque if is_auger else HOLD_TORQUE_NM),
                query=True,
            )
            results = await transport.cycle([lead_cmd, auger_cmd])
            by_id = {r.id: r for r in results}
            elapsed = time.monotonic() - started_monotonic
            print(
                f"  t={elapsed:5.2f}s  "
                f"lead_screw[{_fmt(by_id.get(LEAD_SCREW_ID))}]  "
                f"auger[{_fmt(by_id.get(AUGER_ID))}]"
            )
            await asyncio.sleep(period)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt — stopping motors.")
    finally:
        try:
            await transport.cycle([lead_screw.make_stop(), auger.make_stop()])
            print("both controllers stopped.")
        except Exception as exc:  # pragma: no cover - hardware path
            print(f"WARNING: failed to send make_stop on shutdown: {exc}",
                  file=sys.stderr)

    return 0


def main() -> int:
    args = _parse_args()
    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
