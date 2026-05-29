"""Data-driven analysis protocol — the editable source of truth.

This module is **the one place to edit when the Astrotech lead hands over the
final analysis sequence.** Everything else (the sequencer, the executors, the
ROS interface, the panel) is generic and driven by the data here.

The seed values below are reconstructed from the lead's bench script
``src/astrotech_rover/AnalysisFiles/testreal.py`` (see that folder's README).
That script is bench WIP — most steps are commented out and the volumes / wait
times are rough — so treat all of this as **ASSUMED, confirm with the lead**:
the device→CAN-id map, the site→angle correspondence, and the step lists.

Model
-----
A sequence is an ordered list of ``Step``s. Each step fires one or more
``Action``s (pump or servo) onto the CAN bus, then waits ``wait_s`` seconds
(that's how the bench script works: issue the move, then ``asyncio.sleep`` for
the run/develop time). The sequencer publishes the step description + elapsed
time as it goes, and a cancel hard-aborts (stops every motor).

There is no board feedback (see astrotech_canfd/API_NOTES.md), so this is
commanded-only: the waits ARE the timing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

# --- device kinds ----------------------------------------------------------

BDC = "bdc"      # brushed-DC pump motor: move_motor_forward/reverse(seconds)
SERVO = "servo"  # positioning servo: go_to_position(degrees)

# Pump move duration is a 7-bit field in the CAN frame -> max 127 s per command.
PUMP_MAX_SECONDS = 127


@dataclass(frozen=True)
class Device:
    """A controllable device on the fdcanusb."""
    name: str
    kind: str       # BDC or SERVO
    can_id: int     # board CAN id
    dev_id: int     # motor_id (BDC, 0..5) or servo_id (SERVO, 0..15)


# Device map — logical name -> board/motor wiring. From AnalysisFiles/testreal.py.
# ASSUMED; confirm CAN ids / motor ids with the lead + electrical.
DEVICES: Dict[str, Device] = {
    # DMSO board ("double green", can_id=18)
    "dmsoS1":    Device("dmsoS1", BDC, 18, 0),
    "dmsoS2":    Device("dmsoS2", BDC, 18, 1),
    "ninview":   Device("ninview", BDC, 18, 2),
    "dmsoWaste": Device("dmsoWaste", BDC, 18, 3),
    "heattry1":  Device("heattry1", BDC, 18, 4),
    "heattry2":  Device("heattry2", BDC, 18, 5),
    # H2O board ("one blue", can_id=20)
    "h2oS1":     Device("h2oS1", BDC, 20, 0),
    "h2oS2":     Device("h2oS2", BDC, 20, 1),
    "h2obayers": Device("h2obayers", BDC, 20, 2),
    "bayview":   Device("bayview", BDC, 20, 3),
    "mbview":    Device("mbview", BDC, 20, 4),
    "h2owaste":  Device("h2owaste", BDC, 20, 5),
    # Optics board ("double blue", can_id=21)
    "raman":     Device("raman", BDC, 21, 0),
    "led":       Device("led", BDC, 21, 1),
    "heat1":     Device("heat1", BDC, 21, 2),
    "heat2":     Device("heat2", BDC, 21, 3),
    # Mixing chamber servo (can_id=26, servo_id=15) — same board the mixing-servo
    # panel drives. The analysis driver owns the bus in lab mode (mixing servo
    # mocked), so it can position the chamber as a sequence step.
    "chamber":   Device("chamber", SERVO, 26, 15),
}

# Site -> mixing-chamber angle (deg). ASSUMED correspondence to the measured
# mixing-servo presets (Big Box 110 / Site 1 145 / Site 2 185 from the
# 2026-05-08 bench session). Confirm with the lead which physical site each
# assay step targets, then edit here — the sequences below reference SITES so
# this is the only place the angles live.
SITES: Dict[int, int] = {1: 145, 2: 185}
BIG_BOX_DEG = 110


@dataclass(frozen=True)
class Action:
    kind: str            # BDC or SERVO
    device: str          # key into DEVICES
    magnitude: float     # BDC: seconds (<= PUMP_MAX_SECONDS); SERVO: degrees
    direction: str = ""  # BDC only: "fwd" or "rev"


def pump(device: str, direction: str, seconds: float) -> Action:
    return Action(kind=BDC, device=device, magnitude=seconds, direction=direction)


def servo(device: str, degrees: float) -> Action:
    return Action(kind=SERVO, device=device, magnitude=degrees)


@dataclass(frozen=True)
class Step:
    description: str
    actions: List[Action] = field(default_factory=list)
    wait_s: float = 0.0


@dataclass(frozen=True)
class Sequence:
    sequence_id: int
    name: str
    steps: List[Step]


# --- the sequences (EDIT HERE for the real protocol) -----------------------
# Reconstructed from the commented blocks in AnalysisFiles/testreal.py.
# Durations/waits are rough bench values — confirm with the lead.

_NIN = Sequence(
    sequence_id=1,
    name="Ninhydrin (amino-acid) assay",
    steps=[
        Step("Position chamber at site 1", [servo("chamber", SITES[1])], 2.0),
        Step("Prime DMSO into mixing chamber", [pump("dmsoS1", "fwd", 1)], 1.5),
        Step("Draw DMSO into cuvette",
             [pump("dmsoS1", "fwd", 4), pump("dmsoWaste", "rev", 4)], 4.5),
        Step("Add ninhydrin + DMSO, develop",
             [pump("dmsoS1", "fwd", 4), pump("dmsoWaste", "rev", 4),
              pump("ninview", "rev", 4)], 20.0),
        Step("Flush cuvette",
             [pump("dmsoS1", "fwd", 8), pump("dmsoWaste", "rev", 8)], 9.0),
        Step("Position chamber at site 2", [servo("chamber", SITES[2])], 2.0),
        Step("Site 2: prime",
             [pump("dmsoS2", "fwd", 2), pump("dmsoWaste", "rev", 2)], 2.5),
        Step("Site 2: add ninhydrin, develop",
             [pump("dmsoWaste", "rev", 3), pump("ninview", "rev", 3)], 20.0),
    ],
)

_WATER = Sequence(
    sequence_id=2,
    name="Water-system assay",
    steps=[
        Step("Position chamber at site 1", [servo("chamber", SITES[1])], 2.0),
        Step("Prime water + LED on",
             [pump("h2oS1", "fwd", 1), pump("led", "rev", 15)], 1.5),
        Step("Draw water + Bayer's past Raman",
             [pump("h2oS1", "fwd", 5), pump("h2obayers", "rev", 5),
              pump("raman", "rev", 5)], 5.5),
        Step("Advance + waste",
             [pump("h2oS1", "fwd", 8), pump("h2owaste", "rev", 8),
              pump("h2obayers", "rev", 8)], 8.5),
        Step("Bayer's / methylene-blue views, develop",
             [pump("bayview", "rev", 4), pump("mbview", "rev", 4),
              pump("h2owaste", "rev", 5)], 10.0),
        Step("Position chamber at site 2", [servo("chamber", SITES[2])], 2.0),
        Step("Flush + prep site 2",
             [pump("h2owaste", "rev", 15), pump("h2obayers", "rev", 15),
              pump("h2oS2", "fwd", 15)], 20.0),
        Step("Site 2: Bayer's / MB views",
             [pump("bayview", "rev", 4), pump("mbview", "rev", 4),
              pump("h2owaste", "rev", 5)], 10.0),
    ],
)

SEQUENCES: Dict[int, Sequence] = {s.sequence_id: s for s in (_NIN, _WATER)}


def get_sequence(sequence_id: int) -> Sequence | None:
    return SEQUENCES.get(int(sequence_id))


def referenced_devices() -> List[Device]:
    """Every device any sequence touches — what the executor must build."""
    names = {a.device for seq in SEQUENCES.values() for st in seq.steps for a in st.actions}
    return [DEVICES[n] for n in names if n in DEVICES]
