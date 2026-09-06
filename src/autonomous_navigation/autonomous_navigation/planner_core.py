"""ROS-independent local-planning decisions."""

import math
from typing import Iterable, List, Sequence, Tuple

Point = Tuple[float, float]


def parse_costmap(data: Sequence[float], threshold: float):
    """Return cell costs and currently occupied cells from xyz-style triples."""
    if len(data) % 3:
        raise ValueError("costmap data must contain x, y, cost triples")
    costs = {}
    obstacles = set()
    for index in range(0, len(data), 3):
        cell = (float(data[index]), float(data[index + 1]))
        cost = float(data[index + 2])
        costs[cell] = cost
        if cost > threshold:
            obstacles.add(cell)
    return costs, obstacles


def advance_path(
    position: Point,
    target: Point,
    path: Iterable[Point],
    waypoint: Point,
    threshold: float,
):
    """Advance one reached waypoint and return path, next waypoint, and reached."""
    remaining: List[Point] = list(path)
    if math.dist(position, waypoint) >= threshold:
        return remaining, waypoint, None

    reached = waypoint
    if remaining and remaining[0] == waypoint:
        remaining.pop(0)
    next_waypoint = remaining[0] if remaining else target
    return remaining, next_waypoint, reached
