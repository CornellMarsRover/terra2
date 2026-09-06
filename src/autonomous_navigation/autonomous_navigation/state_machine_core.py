"""ROS-independent autonomy mission decisions."""

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class TargetDecision:
    target: Point
    reached: bool = False
    timed_out: bool = False
    pop_coarse: bool = False
    pop_search: bool = False
    announce_object: bool = False


def far_target(
    position: Point,
    goal: Point,
    current_target: Point,
    coarse_waypoints: Sequence[Point],
    pursuing_object: bool,
    coarse_threshold: float = 2.5,
) -> TargetDecision:
    """Choose a coarse waypoint while the global goal is still far away."""
    if pursuing_object:
        return TargetDecision(current_target)
    remaining = list(coarse_waypoints)
    pop_coarse = bool(
        remaining and math.dist(position, remaining[0]) < coarse_threshold
    )
    if pop_coarse:
        remaining.pop(0)
    return TargetDecision(
        target=remaining[0] if remaining else goal,
        pop_coarse=pop_coarse,
    )


def coordinate_target(
    position: Point,
    goal: Point,
    threshold: float = 2.0,
) -> TargetDecision:
    """Choose a coordinate goal and report when it has been reached."""
    return TargetDecision(goal, reached=math.dist(position, goal) < threshold)
