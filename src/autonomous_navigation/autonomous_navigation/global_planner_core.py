"""ROS-independent persistent mapping and global route decisions.

The local planner searches an 8 m half-width window around the rover and clamps
any goal outside it onto the window edge, so a goal behind a wall reads as a
goal straight through the wall. The costmap also decays every cell on a timer,
which is correct for a sliding obstacle window and wrong for a map: an obstacle
is forgotten a few seconds after it leaves the camera's field of view, so the
rover cannot know a corridor is a dead end until it drives into it again.

This module keeps the second map the rover never had. Cells accumulate evidence
instead of decaying uniformly, so a wall confirmed over several frames persists
after it leaves view while a single-frame false positive fades. A* over that
persistent map produces a route, and the route's first point beyond the local
window becomes the sub-goal handed to the local planner, which keeps owning
obstacle avoidance and smoothing.
"""

import heapq
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

Point = Tuple[float, float]

NEIGHBORS: Tuple[Tuple[int, int], ...] = (
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
)


def quantize(point: Sequence[float], cell_size: float) -> Point:
    """Snap a world point onto the centre of its map cell."""
    if cell_size <= 0.0:
        raise ValueError("cell_size must be positive")
    return (
        round(float(point[0]) / cell_size) * cell_size,
        round(float(point[1]) / cell_size) * cell_size,
    )


@dataclass(frozen=True)
class MapConfig:
    """Evidence thresholds for the persistent map.

    `hit` and `miss` are deliberately asymmetric. A cell the camera reports as
    an obstacle gains confidence faster than a cell it reports as clear loses
    it, because a missed obstacle ends the run and a stale obstacle only costs
    a detour.
    """

    cell_size: float = 0.5
    occupied_threshold: float = 4.0
    hit: float = 3.0
    miss: float = 1.0
    ceiling: float = 12.0
    blocked_cost: float = 4.0

    def __post_init__(self) -> None:
        if self.cell_size <= 0.0:
            raise ValueError("cell_size must be positive")
        if self.hit <= 0.0 or self.miss <= 0.0:
            raise ValueError("hit and miss must be positive")
        if self.ceiling < self.occupied_threshold:
            raise ValueError("ceiling must not be below occupied_threshold")


class WorldMap:
    """A sparse global obstacle map built from repeated local costmaps."""

    def __init__(self, config: Optional[MapConfig] = None) -> None:
        self.config = config or MapConfig()
        self._confidence: Dict[Point, float] = {}
        self._visited: Set[Point] = set()
        self._observed: Set[Point] = set()

    # -- building ---------------------------------------------------------
    def integrate(
        self,
        costs: Mapping[Sequence[float], float],
        observed: Optional[Iterable[Sequence[float]]] = None,
    ) -> None:
        """Fold one local costmap into the persistent map.

        `costs` is the fine-grained local grid. `observed` names the cells the
        camera actually saw this frame; cells inside it that carry no obstacle
        cost lose confidence, and cells outside it are left alone rather than
        being treated as cleared. Leaving `observed` unset means every keyed
        cell was observed.
        """
        blocked_cells: Set[Point] = set()
        seen_cells: Set[Point] = set()
        for cell, cost in costs.items():
            coarse = quantize(cell, self.config.cell_size)
            seen_cells.add(coarse)
            if float(cost) >= self.config.blocked_cost:
                blocked_cells.add(coarse)
        if observed is not None:
            seen_cells = {
                quantize(cell, self.config.cell_size) for cell in observed
            } | blocked_cells

        self._observed |= seen_cells
        for cell in seen_cells:
            if cell in blocked_cells:
                self._confidence[cell] = min(
                    self.config.ceiling,
                    self._confidence.get(cell, 0.0) + self.config.hit,
                )
            else:
                remaining = self._confidence.get(cell, 0.0) - self.config.miss
                if remaining > 0.0:
                    self._confidence[cell] = remaining
                else:
                    self._confidence.pop(cell, None)

    def mark_visited(self, point: Sequence[float]) -> Point:
        """Record a cell the rover has physically driven through."""
        cell = quantize(point, self.config.cell_size)
        self._visited.add(cell)
        self._observed.add(cell)
        self._confidence.pop(cell, None)
        return cell

    # -- reading ----------------------------------------------------------
    def confidence(self, point: Sequence[float]) -> float:
        """Return accumulated obstacle evidence for a point's cell."""
        return self._confidence.get(quantize(point, self.config.cell_size), 0.0)

    def is_blocked(self, point: Sequence[float]) -> bool:
        """Return whether a point's cell holds confirmed obstacle evidence."""
        cell = quantize(point, self.config.cell_size)
        if cell in self._visited:
            return False
        return self._confidence.get(cell, 0.0) >= self.config.occupied_threshold

    def blocked_cells(self) -> Set[Point]:
        """Return every cell currently believed to be an obstacle."""
        return {cell for cell in self._confidence if self.is_blocked(cell)}

    def visited_cells(self) -> Set[Point]:
        """Return every cell the rover has driven through."""
        return set(self._visited)

    def known_cells(self) -> Set[Point]:
        """Return every cell the rover has observed or driven through.

        Anything outside this set is unknown rather than clear, which is what
        makes a frontier a frontier.
        """
        return self._observed | self._visited

    def bounds(self, start: Point, goal: Point, margin: float) -> Tuple[float, float, float, float]:
        """Return the search box covering the map, the rover and the goal."""
        if margin < 0.0:
            raise ValueError("margin must not be negative")
        xs = [start[0], goal[0]]
        ys = [start[1], goal[1]]
        for cell in self.known_cells():
            xs.append(cell[0])
            ys.append(cell[1])
        return (
            min(xs) - margin,
            max(xs) + margin,
            min(ys) - margin,
            max(ys) + margin,
        )


def inflate(blocked: Iterable[Point], cell_size: float, radius: int) -> Set[Point]:
    """Grow obstacle cells by a radius so routes keep a chassis clearance."""
    if cell_size <= 0.0:
        raise ValueError("cell_size must be positive")
    if radius < 0:
        raise ValueError("radius must not be negative")
    grown: Set[Point] = set()
    for cell in blocked:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if math.hypot(dx, dy) <= radius:
                    grown.add(
                        (cell[0] + dx * cell_size, cell[1] + dy * cell_size)
                    )
    return grown


def astar(
    start: Point,
    goal: Point,
    blocked: Set[Point],
    cell_size: float,
    bounds: Tuple[float, float, float, float],
    visited: Optional[Set[Point]] = None,
    visited_discount: float = 0.8,
    max_expansions: int = 40000,
) -> List[Point]:
    """Return an 8-connected route over the persistent map, or an empty list.

    Cells the rover has already driven through cost slightly less, so when the
    only way on is backwards the route prefers ground known to be drivable
    over unknown ground that merely looks shorter.
    """
    if cell_size <= 0.0:
        raise ValueError("cell_size must be positive")
    if not 0.0 < visited_discount <= 1.0:
        raise ValueError("visited_discount must be in (0, 1]")
    if max_expansions < 1:
        raise ValueError("max_expansions must be positive")

    min_x, max_x, min_y, max_y = bounds
    start_cell = quantize(start, cell_size)
    goal_cell = quantize(goal, cell_size)
    known = visited or set()

    def in_bounds(cell: Point) -> bool:
        return min_x <= cell[0] <= max_x and min_y <= cell[1] <= max_y

    if goal_cell in blocked or not in_bounds(goal_cell):
        return []

    def heuristic(cell: Point) -> float:
        return math.dist(cell, goal_cell)

    open_heap: List[Tuple[float, float, Point]] = [
        (heuristic(start_cell), 0.0, start_cell)
    ]
    came_from: Dict[Point, Point] = {}
    best: Dict[Point, float] = {start_cell: 0.0}
    closed: Set[Point] = set()
    expansions = 0

    while open_heap:
        _, cost, cell = heapq.heappop(open_heap)
        if cell in closed:
            continue
        if cell == goal_cell:
            return _reconstruct(came_from, cell)
        closed.add(cell)
        expansions += 1
        if expansions >= max_expansions:
            return []
        for dx, dy in NEIGHBORS:
            neighbor = (cell[0] + dx * cell_size, cell[1] + dy * cell_size)
            if neighbor in closed or neighbor in blocked or not in_bounds(neighbor):
                continue
            step = math.hypot(dx, dy) * cell_size
            if neighbor in known:
                step *= visited_discount
            candidate = cost + step
            if candidate < best.get(neighbor, math.inf):
                best[neighbor] = candidate
                came_from[neighbor] = cell
                heapq.heappush(
                    open_heap, (candidate + heuristic(neighbor), candidate, neighbor)
                )
    return []


def _reconstruct(came_from: Dict[Point, Point], cell: Point) -> List[Point]:
    """Walk the parent chain back to the start and return it forwards."""
    route = [cell]
    while cell in came_from:
        cell = came_from[cell]
        route.append(cell)
    route.reverse()
    return route


def route_subgoal(route: Sequence[Point], lookahead: float) -> Optional[Point]:
    """Return the first route point at least `lookahead` from its start.

    The sub-goal is deliberately not the next cell. Handing the local planner a
    point one cell ahead would overwrite the avoidance it is doing inside its
    own window; handing it a point a window away leaves it free to deviate and
    still converge.
    """
    if lookahead <= 0.0:
        raise ValueError("lookahead must be positive")
    if not route:
        return None
    origin = route[0]
    for point in route[1:]:
        if math.dist(origin, point) >= lookahead:
            return point
    return route[-1]


def frontier_subgoal(
    start: Point,
    goal: Point,
    blocked: Set[Point],
    cell_size: float,
    bounds: Tuple[float, float, float, float],
    known: Optional[Set[Point]] = None,
    visited: Optional[Set[Point]] = None,
) -> Optional[Point]:
    """Return the next place worth driving to when no route to the goal exists.

    That is the normal case on the first approach to a waypoint, when most of
    the world is unseen, and it is also what happens inside a dead end.

    A frontier cell is free, reachable, and touching a cell the rover has never
    observed. Choosing by that definition rather than by distance alone is what
    makes the rover reverse out of a pocket: once a pocket is fully observed,
    every cell that still touches the unknown is back at its mouth, so the
    nearest frontier is behind the rover even though the pocket's far wall is
    closer to the goal.
    """
    reachable = reachable_cells(start, blocked, cell_size, bounds)
    start_cell = quantize(start, cell_size)
    reachable.discard(start_cell)
    if not reachable:
        return None
    goal_cell = quantize(goal, cell_size)
    seen = known or set()
    driven = visited or set()

    frontier = {
        cell
        for cell in reachable
        if any(
            (cell[0] + dx * cell_size, cell[1] + dy * cell_size) not in seen
            for dx, dy in NEIGHBORS
        )
    }
    candidates = frontier or reachable
    return min(
        candidates,
        key=lambda cell: (
            math.dist(cell, goal_cell),
            0 if cell in driven else 1,
            cell,
        ),
    )


def reachable_cells(
    start: Point,
    blocked: Set[Point],
    cell_size: float,
    bounds: Tuple[float, float, float, float],
    max_cells: int = 40000,
) -> Set[Point]:
    """Flood fill the free cells connected to the rover's own cell."""
    if cell_size <= 0.0:
        raise ValueError("cell_size must be positive")
    if max_cells < 1:
        raise ValueError("max_cells must be positive")
    min_x, max_x, min_y, max_y = bounds
    start_cell = quantize(start, cell_size)
    if start_cell in blocked:
        return set()
    seen = {start_cell}
    queue = [start_cell]
    while queue and len(seen) < max_cells:
        cell = queue.pop()
        for dx, dy in NEIGHBORS:
            neighbor = (cell[0] + dx * cell_size, cell[1] + dy * cell_size)
            if neighbor in seen or neighbor in blocked:
                continue
            if not (min_x <= neighbor[0] <= max_x and min_y <= neighbor[1] <= max_y):
                continue
            seen.add(neighbor)
            queue.append(neighbor)
            if len(seen) >= max_cells:
                return seen
    return seen


@dataclass(frozen=True)
class GlobalPlan:
    """What the node publishes, plus why, so telemetry can explain a detour."""

    subgoal: Point
    route: Tuple[Point, ...]
    reason: str

    @property
    def replanned(self) -> bool:
        """Return whether the sub-goal differs from the mission waypoint."""
        return self.reason != "direct"


def plan(
    start: Sequence[float],
    goal: Sequence[float],
    world: WorldMap,
    lookahead: float = 6.0,
    inflation_radius: int = 1,
    margin: float = 4.0,
    arrival: float = 1.0,
) -> GlobalPlan:
    """Return the sub-goal the local planner should drive toward next.

    Falls back in a fixed order: hand the waypoint straight through when the
    map knows nothing blocking, otherwise route around what it knows, and
    otherwise drive at the frontier that shortens the remaining distance. The
    reason is carried out so a detour is explainable after the run instead of
    looking like the rover wandering off.
    """
    if arrival <= 0.0:
        raise ValueError("arrival must be positive")
    cell_size = world.config.cell_size
    start_point = (float(start[0]), float(start[1]))
    goal_point = (float(goal[0]), float(goal[1]))

    if math.dist(start_point, goal_point) <= arrival:
        return GlobalPlan(goal_point, (goal_point,), "direct")

    blocked = inflate(world.blocked_cells(), cell_size, inflation_radius)
    blocked -= world.visited_cells()
    bounds = world.bounds(start_point, goal_point, margin)

    if not blocked:
        return GlobalPlan(goal_point, (start_point, goal_point), "direct")

    route = astar(
        start_point,
        goal_point,
        blocked,
        cell_size,
        bounds,
        visited=world.visited_cells(),
    )
    if route:
        return GlobalPlan(route_subgoal(route, lookahead), tuple(route), "routed")

    frontier = frontier_subgoal(
        start_point,
        goal_point,
        blocked,
        cell_size,
        bounds,
        known=world.known_cells(),
        visited=world.visited_cells(),
    )
    if frontier is None:
        return GlobalPlan(start_point, (start_point,), "trapped")
    return GlobalPlan(frontier, (start_point, frontier), "frontier")
