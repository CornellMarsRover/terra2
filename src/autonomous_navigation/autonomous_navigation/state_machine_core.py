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


def search_target(
    position: Point,
    current_target: Point,
    search_waypoints: Sequence[Point],
    threshold: float = 2.0,
) -> TargetDecision:
    """Choose the next object-search waypoint or report search exhaustion."""
    if not search_waypoints:
        return TargetDecision(current_target, timed_out=True)
    next_target = search_waypoints[0]
    if math.dist(position, next_target) >= threshold:
        return TargetDecision(next_target, announce_object=True)
    if len(search_waypoints) == 1:
        return TargetDecision(current_target, timed_out=True, pop_search=True)
    return TargetDecision(search_waypoints[1], pop_search=True)


def object_target(
    position: Point,
    target: Point,
    threshold: float = 1.5,
) -> TargetDecision:
    """Continue toward a detected object until its standoff is reached."""
    return TargetDecision(target, reached=math.dist(position, target) < threshold)


def search_waypoints(
    center: Point,
    angle_step: float,
    radius_step: float,
    maximum_radius: float,
):
    """Generate the legacy expanding search pattern around a target."""
    points = []
    radius = radius_step
    angle = 0.0
    while radius < maximum_radius:
        if angle > 180:
            angle = -(360 - angle)
        radians = math.radians(angle)
        dx = radius * (math.cos(radians) - math.sin(radians))
        dy = radius * (math.sin(radians) + math.sin(radians))
        points.append((center[0] + dx, center[1] + dy))
        angle += -angle_step if angle < 0 else angle_step
        radius += radius_step
    return points
