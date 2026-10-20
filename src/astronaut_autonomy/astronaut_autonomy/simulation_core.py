"""Deterministic hardware-free astronaut perception scenario."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SimulatedPerception:
    gesture: str
    command: str
    confidence: float
    astronaut_detected: bool
    distance_valid: bool
    depth_status: str
    distance_m: float
    bearing_deg: float
    label: str


SCENARIO_DURATION_S = 15.0


def scripted_perception(elapsed_s: float) -> SimulatedPerception:
    """Generate gestures, motion, repeats, invalid depth, and lost tracking."""
    if not math.isfinite(elapsed_s):
        raise ValueError("elapsed time must be finite")
    time_s = max(0.0, elapsed_s) % SCENARIO_DURATION_S

    if time_s < 1.0:
        return _sample("NONE", detected=True, distance=4.0, bearing=20.0,
                       label="idle")
    if time_s < 4.0:
        phase = (time_s - 1.0) / 3.0
        return _sample(
            "FOLLOW",
            distance=4.5 - 2.1 * phase,
            bearing=22.0 * (1.0 - phase),
            label="follow_moving_and_repeated",
        )
    if time_s < 5.0:
        return _sample(
            "FOLLOW",
            command="NONE",
            valid=False,
            distance=math.nan,
            bearing=math.nan,
            label="invalid_depth",
        )
    if time_s < 6.0:
        return _sample(
            "NONE",
            detected=False,
            valid=False,
            distance=math.nan,
            bearing=math.nan,
            label="lost_tracking",
        )
    if time_s < 7.5:
        return _sample("STAY", detected=True, distance=2.5, bearing=0.0,
                       label="stay")
    if time_s < 10.5:
        phase = (time_s - 7.5) / 3.0
        return _sample(
            "COME",
            distance=3.5 - 2.1 * phase,
            bearing=-18.0 * (1.0 - phase),
            label="come_moving",
        )
    if time_s < 12.0:
        return _sample("FETCH", distance=1.8, bearing=5.0, label="fetch")
    if time_s < 13.0:
        return _sample("NONE", detected=True, distance=1.8, bearing=0.0,
                       label="neutral_rearm")
    return _sample("GIVE", detected=True, distance=1.5, bearing=0.0,
                   label="give")


def _sample(
    gesture: str,
    *,
    command: str | None = None,
    confidence: float = 0.92,
    detected: bool = True,
    valid: bool = True,
    distance: float = 2.0,
    bearing: float = 0.0,
    label: str,
) -> SimulatedPerception:
    return SimulatedPerception(
        gesture=gesture,
        command=gesture if command is None else command,
        confidence=confidence,
        astronaut_detected=detected,
        distance_valid=valid,
        depth_status="VALID" if valid else (
            "UNAVAILABLE" if detected else "NO_POSE"
        ),
        distance_m=distance,
        bearing_deg=bearing,
        label=label,
    )
