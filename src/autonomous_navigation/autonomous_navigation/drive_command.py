"""Convert autonomy decisions to normalized shared-drive commands."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ChassisCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


def _clamp(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("command values must be finite")
    return max(-1.0, min(1.0, value))


def inputs_fresh(now_s, pose_time_s, waypoint_time_s, timeout_s: float) -> bool:
    """Require recent pose and waypoint samples before allowing rover motion."""
    if timeout_s <= 0.0 or not math.isfinite(timeout_s):
        raise ValueError("timeout_s must be finite and positive")
    if pose_time_s is None or waypoint_time_s is None:
        return False
    ages = (float(now_s) - float(pose_time_s), float(now_s) - float(waypoint_time_s))
    return all(math.isfinite(age) and 0.0 <= age <= timeout_s for age in ages)


def waypoint_reached(position, waypoint, tolerance: float = 0.3) -> bool:
    """Return whether a planar waypoint is within the arrival tolerance."""
    if tolerance < 0.0:
        raise ValueError("tolerance must not be negative")
    return math.dist(position, waypoint) <= tolerance


def forward_heading_command(
    linear_x: float,
    heading_error_deg: float,
    max_heading_error_deg: float = 35.0,
) -> ChassisCommand:
    """Map forward demand and heading error to ``/cmd_vel_drives`` axes."""
    if max_heading_error_deg <= 0.0:
        raise ValueError("max_heading_error_deg must be positive")
    heading = max(
        -max_heading_error_deg,
        min(max_heading_error_deg, float(heading_error_deg)),
    )
    return ChassisCommand(_clamp(linear_x), angular_z=_clamp(math.radians(heading)))


def point_turn_command(angular_z: float) -> ChassisCommand:
    """Map a signed turn demand to the shared drive command."""
    return ChassisCommand(angular_z=_clamp(angular_z))
