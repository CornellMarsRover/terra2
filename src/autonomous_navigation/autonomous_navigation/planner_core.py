"""ROS-independent local-planning decisions."""

import math
from typing import Iterable, List, Sequence, Tuple

Point = Tuple[float, float]


def record_segment_observation(counts, segment, blocked, confirmations=3):
    """Track consecutive blocked observations and report confirmed blockage."""
    if confirmations < 1:
        raise ValueError("confirmations must be positive")
    updated = dict(counts)
    if not blocked:
        updated.pop(segment, None)
        return updated, False
    updated[segment] = updated.get(segment, 0) + 1
    return updated, updated[segment] >= confirmations


def nearest_clear_goal(goal: Point, bounds, is_blocked, step: float) -> Point:
    """Return the nearest in-bounds grid point that is not blocked."""
    if step <= 0.0:
        raise ValueError("step must be positive")
    min_x, max_x, min_y, max_y = bounds
    origin = (
        max(min(float(goal[0]), max_x), min_x),
        max(min(float(goal[1]), max_y), min_y),
    )
    max_radius = math.ceil(max(max_x - min_x, max_y - min_y) / step)
    for radius in range(max_radius + 1):
        offsets = range(-radius, radius + 1)
        candidates = (
            (origin[0] + dx * step, origin[1] + dy * step)
            for dx in offsets
            for dy in offsets
            if max(abs(dx), abs(dy)) == radius
        )
        for candidate in sorted(
            candidates, key=lambda point: (math.dist(origin, point), point)
        ):
            in_bounds = (
                min_x <= candidate[0] <= max_x
                and min_y <= candidate[1] <= max_y
            )
            if in_bounds and not is_blocked(candidate):
                return candidate
    return origin


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


def neighbor_cost(costs, cell: Point, cell_size: float, radius: int) -> float:
    """Return inverse-distance-weighted costs around a grid cell."""
    total = 0.0
    for dx in range(-radius, radius):
        for dy in range(-radius, radius):
            if dx == 0 and dy == 0:
                continue
            distance = math.hypot(dx * cell_size, dy * cell_size)
            cost = costs.get(
                (cell[0] + dx * cell_size, cell[1] + dy * cell_size), 0.0
            )
            total += (0.0 if cost == 1 else cost) / distance
    return total


def segment_cost(
    costs,
    start: Point,
    end: Point,
    cell_size: float = 0.25,
    neighbor_radius: int = 4,
):
    """Return maximum and accumulated sampled cost along a segment."""
    distance = math.dist(start, end)
    if distance < 1e-6:
        return 0.0, 0.0
    samples = max(1, int(round(distance / (cell_size * 2))))
    values = []
    for index in range(samples + 1):
        ratio = index / samples
        point = (
            start[0] + ratio * (end[0] - start[0]),
            start[1] + ratio * (end[1] - start[1]),
        )
        cell = tuple(round(value / cell_size) * cell_size for value in point)
        values.append(
            costs.get(cell, 0.0)
            + neighbor_cost(costs, cell, cell_size, neighbor_radius)
        )
    return max(values), sum(values)
