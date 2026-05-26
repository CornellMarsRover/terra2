#!/usr/bin/env python3
"""Read-only DRV8323 gate-driver diagnostic for the Astrotech auger stack.

Two controllers on the same fdcanusb transport:

    id=15  lead_screw  (vertical motion, known healthy)
    id=16  auger       (rotational drill, suspected hardware fault)

Why this exists
---------------
The auger controller (id=16) trips ``fault=33`` ("gate driver fault") within
~250 ms of any motion command, with encoder validity faults firing first
(``98``/``102``).  The moteus_guide.pdf (read late in the previous session)
flags ``fault=33`` as "most likely a hardware issue" -- recoverable case is
UVLO (undervoltage lockout); otherwise a blown MOSFET / dead DRV8323.

The same diagnostic that ``moteus_tool`` runs internally when it catches
fault=33 during calibration (line 1249 of ``moteus_tool.py``) is::

    drv8323 = await stream.read_data("drv8323")

This script does that read on **both** controllers and prints a side-by-side
comparison so we can see which DRV8323 status bits differ between the
healthy lead screw and the suspect auger.

It also dumps the bus-voltage-bearing fields from ``servo_stats`` so we can
compare ``filt_bus_V`` between the two controllers (a UVLO would show up as
a sagging or low bus voltage on the auger relative to the lead screw).

Strict guarantees
-----------------
* No ``transport.cycle()`` motion commands -- nothing is ever driven.
* No ``conf set`` / ``conf write`` -- nothing is written to flash.
* The only writes through the diagnostic stream are ``tel fmt drv8323 0`` and
  ``tel get drv8323`` (volatile, identical to what ``moteus_tool --read drv8323``
  does); these do not persist across power cycle.
* Both controllers must be in ``mode=0`` (stopped) at start; if either is in
  any other mode this script aborts without touching anything.
* ``Ctrl-C`` aborts cleanly via the asyncio loop (no ``kill -KILL`` needed --
  see the procedural lessons in tonight's session handoff).

Tip
---
When piping to ``tee`` for a session log, run via ``python3 -u`` so progress
prints actually appear in the log in real time::

    python3 -u src/astrotech_rover/scripts/auger_drv8323_status.py 2>&1 | tee /tmp/auger_drv8323.log
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from typing import Any

import moteus
from moteus.moteus import namedtuple_to_dict

LEAD_SCREW_ID = 15
AUGER_ID = 16

PING_TIMEOUT_S = 1.0
DIAG_TIMEOUT_S = 5.0  # diagnostic stream needs more wall time than a ping.

# servo_stats has dozens of fields; the ones below are the load/supply-related
# ones relevant to a UVLO investigation. Anything else from servo_stats is
# noisy for this comparison (encoder counts, control-loop intermediates, etc.)
# and would obscure the signal.
SERVO_STATS_BUS_FIELDS = (
    "mode",
    "fault",
    "filt_bus_V",
    "filt_1ms_motor_current_A",
    "bus_V",
    "temperature",
    "fet_temperature",
    "motor_temperature",
)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--lead-screw-id",
        type=int,
        default=LEAD_SCREW_ID,
        help=f"CAN id of the lead screw controller (default {LEAD_SCREW_ID}).",
    )
    p.add_argument(
        "--auger-id",
        type=int,
        default=AUGER_ID,
        help=f"CAN id of the auger controller (default {AUGER_ID}).",
    )
    p.add_argument(
        "--json",
        action="store_true",
        help="Emit a single JSON document on stdout instead of the human-readable diff.",
    )
    return p.parse_args()


async def _ping(transport: moteus.Transport, controller: moteus.Controller,
                label: str) -> dict[str, Any] | None:
    """Verify the controller is reachable and currently idle.

    Returns the parsed query values dict, or None if the controller is not
    reachable / not in mode=0.
    """
    try:
        result = await asyncio.wait_for(
            transport.cycle([controller.make_query()]), timeout=PING_TIMEOUT_S
        )
    except asyncio.TimeoutError:
        print(
            f"  {label} (id={controller.id}): no reply within "
            f"{PING_TIMEOUT_S:.1f} s",
            file=sys.stderr,
        )
        return None
    except Exception as exc:  # pragma: no cover - hardware path
        print(f"  {label} (id={controller.id}): transport error: {exc}",
              file=sys.stderr)
        return None
    if not result:
        print(f"  {label} (id={controller.id}): no reply on the bus",
              file=sys.stderr)
        return None
    values = result[0].values
    mode = int(values.get(moteus.Register.MODE, -1))
    fault = int(values.get(moteus.Register.FAULT, -1))
    print(f"  {label} (id={controller.id}): mode={mode} fault={fault}")
    if mode != 0:
        print(
            f"    refusing to read diagnostics: controller is in mode={mode}, "
            "not 0 (stopped). Power-cycle or send a make_stop and retry.",
            file=sys.stderr,
        )
        return None
    return values


async def _read_named(stream: moteus.Stream, name: str) -> dict[str, Any]:
    """Read a named telemetry channel (e.g. 'drv8323') and return as a dict."""
    raw = await asyncio.wait_for(stream.read_data(name), timeout=DIAG_TIMEOUT_S)
    return namedtuple_to_dict(raw)


async def _gather_for_controller(transport: moteus.Transport,
                                 can_id: int) -> dict[str, Any]:
    """Run all read-only diagnostics for one controller. No motion, no flash."""
    controller = moteus.Controller(id=can_id, transport=transport)
    stream = moteus.Stream(controller)
    drv = await _read_named(stream, "drv8323")
    servo = await _read_named(stream, "servo_stats")
    bus = {k: servo.get(k) for k in SERVO_STATS_BUS_FIELDS if k in servo}
    return {"drv8323": drv, "servo_stats_bus": bus}


def _flatten(prefix: str, obj: Any, out: dict[str, Any]) -> None:
    """Flatten nested dict/list into dotted keys for diff display."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            _flatten(f"{prefix}.{k}" if prefix else str(k), v, out)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            _flatten(f"{prefix}[{i}]", v, out)
    else:
        out[prefix] = obj


def _emit_diff(lead_screw: dict[str, Any], auger: dict[str, Any]) -> None:
    print("\n=== DRV8323 status ===")
    print("(true/non-zero gate-driver fault bits indicate a hardware problem)")
    _emit_section_diff(lead_screw["drv8323"], auger["drv8323"])

    print("\n=== servo_stats (bus + temperature subset) ===")
    print("(big delta in filt_bus_V / bus_V points at a power-supply issue / UVLO)")
    _emit_section_diff(lead_screw["servo_stats_bus"], auger["servo_stats_bus"])


def _emit_section_diff(a: dict[str, Any], b: dict[str, Any]) -> None:
    flat_a: dict[str, Any] = {}
    flat_b: dict[str, Any] = {}
    _flatten("", a, flat_a)
    _flatten("", b, flat_b)
    keys = sorted(set(flat_a) | set(flat_b))
    width = max((len(k) for k in keys), default=4)
    header = f"  {'field':<{width}}  {'lead_screw':>14}  {'auger':>14}  diff"
    print(header)
    print("  " + "-" * (width + 2 + 14 + 2 + 14 + 2 + 4))
    for k in keys:
        va = flat_a.get(k, "<missing>")
        vb = flat_b.get(k, "<missing>")
        marker = "" if va == vb else "  <-- DIFFERS"
        # Highlight any obviously bad bit (truthy boolean / non-zero int) on the auger.
        if marker and isinstance(vb, (bool, int)) and vb:
            marker = "  <-- DIFFERS *** auger non-zero"
        print(f"  {k:<{width}}  {_fmt_val(va):>14}  {_fmt_val(vb):>14}{marker}")


def _fmt_val(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:.4g}"
    return str(v)


async def main_async(args: argparse.Namespace) -> int:
    transport = moteus.get_singleton_transport()

    print(f"transport: {type(transport).__name__}")
    print("pinging controllers (must both be mode=0)...")
    ls_ping = await _ping(
        transport, moteus.Controller(id=args.lead_screw_id, transport=transport),
        "lead_screw")
    a_ping = await _ping(
        transport, moteus.Controller(id=args.auger_id, transport=transport),
        "auger     ")
    if ls_ping is None or a_ping is None:
        print("\nrefusing to read diagnostics: controllers not both reachable + idle.",
              file=sys.stderr)
        return 2

    # Read sequentially -- the diagnostic stream is multi-round-trip per call,
    # and interleaving two controllers' diagnostic streams over a shared
    # transport is asking for trouble (it works, but only fragilely).
    print("\nreading lead_screw drv8323 + servo_stats...")
    lead_screw_data = await _gather_for_controller(transport, args.lead_screw_id)
    print("reading auger drv8323 + servo_stats...")
    auger_data = await _gather_for_controller(transport, args.auger_id)

    if args.json:
        print(json.dumps(
            {"lead_screw": lead_screw_data, "auger": auger_data},
            indent=2, default=str,
        ))
        return 0

    _emit_diff(lead_screw_data, auger_data)
    return 0


def main() -> int:
    args = _parse_args()
    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        # Clean SIGINT path -- asyncio unwinds, no kill -KILL needed.
        return 130


if __name__ == "__main__":
    sys.exit(main())
