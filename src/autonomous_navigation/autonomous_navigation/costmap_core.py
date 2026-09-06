"""ROS-independent costmap projection and update decisions."""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class GridObservation:
    cell: Point
    height: float


def project_point(
    point: Sequence[float],
    pose: Point,
    rotation: Sequence[Sequence[float]],
    real: bool,
    camera_height: float,
    min_depth: float,
    max_depth: float,
    cell_size: float,
) -> Optional[GridObservation]:
    """Project one real or simulated camera point into a global grid cell."""
    if real:
        forward, left = point[0], point[1]
        height = camera_height + point[2]
        distance = math.hypot(forward, left)
        bearing = math.degrees(math.atan2(left, forward))
        if distance > max_depth or distance < min_depth or abs(bearing) > 45.0:
            return None
    else:
        forward, left = point[2], point[0]
        height = camera_height - point[1]
        if forward > max_depth or forward < min_depth:
            return None

    rotated_forward = rotation[0][0] * forward + rotation[0][1] * left
    rotated_left = rotation[1][0] * forward + rotation[1][1] * left
    global_forward = rotated_forward + pose[0]
    global_left = (rotated_left if real else -rotated_left) + pose[1]
    cell = (
        round(global_forward / cell_size) * cell_size,
        round(global_left / cell_size) * cell_size,
    )
    return GridObservation(cell, height)


def observed_cost(
    current: float,
    height: float,
    ground_threshold: float,
    obstacle_threshold: float,
    clearance_height: float,
    maximum: float,
) -> Optional[float]:
    """Return an updated obstacle cost, or None when the cell is traversable."""
    if ground_threshold < height < obstacle_threshold or height > clearance_height:
        return None
    increment = 7 if height > 0.5 else 2
    return min(maximum, current + increment)


def decay_costs(costs, amount: float, eligible=None):
    """Return a decayed copy, preserving cells outside an optional eligible set."""
    if amount < 0:
        raise ValueError("decay amount cannot be negative")
    eligible_cells = set(costs) if eligible is None else set(eligible)
    decayed = {}
    for cell, cost in costs.items():
        value = max(0, cost - amount) if cell in eligible_cells else cost
        if value > 0:
            decayed[cell] = value
    return decayed
