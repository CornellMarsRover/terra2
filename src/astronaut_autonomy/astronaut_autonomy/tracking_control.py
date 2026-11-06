"""Hardware-independent FOLLOW and COME velocity control."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TrackingConfig:
    follow_distance_m: float = 2.0
    come_distance_m: float = 1.25
    distance_deadband_m: float = 0.15
    bearing_deadband_deg: float = 4.0
    rotate_in_place_deg: float = 30.0
    linear_gain: float = 0.55
    angular_gain: float = 1.2
    maximum_linear: float = 0.50
    maximum_angular: float = 0.60

    def __post_init__(self):
        positive = (
            self.follow_distance_m,
            self.come_distance_m,
            self.linear_gain,
            self.angular_gain,
            self.maximum_linear,
            self.maximum_angular,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("tracking distances, gains, and limits must be positive")
        nonnegative = (self.distance_deadband_m, self.bearing_deadband_deg)
        if not all(
            math.isfinite(value) and value >= 0.0 for value in nonnegative
        ):
            raise ValueError("tracking deadbands must be finite and nonnegative")
        if (
            not math.isfinite(self.rotate_in_place_deg)
            or self.rotate_in_place_deg <= self.bearing_deadband_deg
            or self.rotate_in_place_deg > 180.0
        ):
            raise ValueError("rotate-in-place angle must exceed the bearing deadband")
        if self.maximum_linear > 1.0 or self.maximum_angular > 1.0:
            raise ValueError("normalized velocity limits cannot exceed 1.0")


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0
    reason: str = "stopped"

    @property
    def stopped(self) -> bool:
        return self.linear_x == 0.0 and self.angular_z == 0.0


def stopped_command(reason: str) -> VelocityCommand:
    return VelocityCommand(reason=reason)


def tracking_velocity(
    distance_m: float,
    bearing_deg: float,
    target_distance_m: float,
    config: TrackingConfig,
    *,
    perception_valid: bool = True,
    emergency_stop: bool = False,
) -> VelocityCommand:
    """Calculate a normalized rover command toward an astronaut.

    Positive bearing is left in the ZED frame and therefore produces a
    positive angular command. The controller never reverses toward a nearby
    person: at or inside the target standoff it stops.
    """
    if emergency_stop:
        return stopped_command("emergency_stop")
    values = (distance_m, bearing_deg, target_distance_m)
    if not perception_valid or not all(math.isfinite(v) for v in values):
        return stopped_command("invalid_perception")
    if distance_m <= 0.0 or target_distance_m <= 0.0:
        return stopped_command("invalid_perception")

    bearing_rad = math.radians(bearing_deg)
    angular = config.angular_gain * bearing_rad
    angular = max(-config.maximum_angular, min(config.maximum_angular, angular))
    if abs(bearing_deg) <= config.bearing_deadband_deg:
        angular = 0.0

    if abs(bearing_deg) >= config.rotate_in_place_deg:
        return VelocityCommand(angular_z=angular, reason="aligning")

    distance_error = distance_m - target_distance_m
    if distance_error <= config.distance_deadband_m:
        return VelocityCommand(angular_z=angular, reason="at_standoff")

    linear = min(config.maximum_linear, config.linear_gain * distance_error)
    # Reduce forward speed while the target is off-axis, without changing the
    # configured hard velocity limit.
    linear *= max(0.0, math.cos(bearing_rad))
    return VelocityCommand(
        linear_x=linear,
        angular_z=angular,
        reason="approaching",
    )
