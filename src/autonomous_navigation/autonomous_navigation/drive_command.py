"""Convert autonomy decisions to normalized shared-drive commands."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ChassisCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


def _clamp(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


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
