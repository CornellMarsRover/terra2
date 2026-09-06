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


def path_is_dense(path: Sequence[Point], distance_threshold: float = 1.0) -> bool:
    """Return whether any three consecutive points form two short segments."""
    return any(
        math.dist(path[index], path[index + 1]) < distance_threshold
        and math.dist(path[index + 1], path[index + 2]) < distance_threshold
        for index in range(len(path) - 2)
    )


def perpendicular_distance(point: Point, start: Point, end: Point) -> float:
    """Calculate a point's perpendicular distance from an infinite line."""
    if start == end:
        return math.dist(point, start)
    x0, y0 = point
    x1, y1 = start
    x2, y2 = end
    numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    return numerator / math.hypot(y2 - y1, x2 - x1)


def simplify_path(points: Sequence[Point], epsilon: float) -> List[Point]:
    """Simplify a path with the Ramer-Douglas-Peucker algorithm."""
    if len(points) < 3:
        return list(points)
    distances = [
        perpendicular_distance(point, points[0], points[-1])
        for point in points[1:-1]
    ]
    max_distance = max(distances)
    if max_distance <= epsilon:
        return [points[0], points[-1]]
    split = distances.index(max_distance) + 1
    left = simplify_path(points[: split + 1], epsilon)
    right = simplify_path(points[split:], epsilon)
    return left[:-1] + right
