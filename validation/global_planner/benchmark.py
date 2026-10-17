#!/usr/bin/env python3
"""Measure the global planner against the window-only planner it sits on top of.

Both planners get the same obstacle maps and the same 8-connected A*. The only
difference is what they are allowed to remember and what goal they are given:

  window  reproduces today's behaviour. It searches a half-width window around
          the rover, and any goal outside that window is clamped onto its edge,
          which is what `nearest_clear_goal` does in local_planner.py today.
  global  routes over the persistent map and feeds the local planner a sub-goal
          that is known to be reachable.

Run from the repo root:

    PYTHONPATH=src/autonomous_navigation python3 validation/global_planner/benchmark.py
"""

import math
import sys

from autonomous_navigation.global_planner_core import (
    MapConfig,
    WorldMap,
    astar,
    plan,
)

CELL = 1.0
WINDOW = 8.0
STEP_LIMIT = 300
ARRIVAL = 1.0


def wall(x, y_from, y_to):
    return {(float(x), float(y)) for y in range(y_from, y_to + 1)}


def horizontal(y, x_from, x_to):
    return {(float(x), float(y)) for x in range(x_from, x_to + 1)}


SCENARIOS = {
    "single wall, 12 m wide": {
        "obstacles": wall(6, -6, 6),
        "start": (0.0, 0.0),
        "goal": (12.0, 0.0),
        "bounds": (-4.0, 16.0, -10.0, 10.0),
    },
    "S maze, two offset walls": {
        "obstacles": wall(5, -8, 2) | wall(10, -2, 8),
        "start": (0.0, 0.0),
        "goal": (14.0, 0.0),
        "bounds": (-4.0, 18.0, -10.0, 10.0),
    },
    "dead-end pocket on the direct line": {
        "obstacles": (
            wall(8, -3, 3)
            | horizontal(3, 3, 8)
            | horizontal(-3, 3, 8)
            | wall(14, -10, 10)
            | horizontal(10, -4, 20)
            | horizontal(-10, -4, 20)
        ),
        "start": (0.0, 0.0),
        "goal": (12.0, 0.0),
        "bounds": (-4.0, 20.0, -11.0, 11.0),
    },
}


def run_window(obstacles, start, goal, bounds):
    """Drive the window-only planner: clamp the goal into the local box."""
    position = start
    travelled = 0.0
    for step in range(STEP_LIMIT):
        if math.dist(position, goal) <= ARRIVAL:
            return True, travelled, step
        local = (
            max(position[0] - WINDOW, min(goal[0], position[0] + WINDOW)),
            max(position[1] - WINDOW, min(goal[1], position[1] + WINDOW)),
        )
        box = (
            max(bounds[0], position[0] - WINDOW),
            min(bounds[1], position[0] + WINDOW),
            max(bounds[2], position[1] - WINDOW),
            min(bounds[3], position[1] + WINDOW),
        )
        route = astar(position, local, obstacles, CELL, box)
        if len(route) < 2:
            return False, travelled, step
        travelled += math.dist(position, route[1])
        position = route[1]
    return False, travelled, STEP_LIMIT


def run_global(obstacles, start, goal, bounds):
    """Drive the global planner, revealing obstacles only within sensor range."""
    sensor_range = 5.0
    world = WorldMap(MapConfig(cell_size=CELL, hit=3.0, miss=1.0))
    position = start
    travelled = 0.0
    for step in range(STEP_LIMIT):
        world.mark_visited(position)
        visible = {
            cell: 10.0
            for cell in obstacles
            if math.dist(cell, position) <= sensor_range
        }
        seen = [
            (position[0] + dx * CELL, position[1] + dy * CELL)
            for dx in range(-5, 6)
            for dy in range(-5, 6)
            if math.hypot(dx, dy) * CELL <= sensor_range
        ]
        for _ in range(2):
            world.integrate(visible, observed=seen)
        if math.dist(position, goal) <= ARRIVAL:
            return True, travelled, step
        result = plan(
            position, goal, world, lookahead=2.0, inflation_radius=0, margin=4.0
        )
        if result.reason == "trapped":
            return False, travelled, step
        position, moved = _advance(position, result.subgoal)
        if moved == 0.0:
            return False, travelled, step
        travelled += moved
    return False, travelled, STEP_LIMIT


def _advance(position, subgoal, limit=CELL):
    """Move one cell toward the sub-goal, never teleport onto it.

    The sub-goal can be metres away, and on an empty map it is the waypoint
    itself. Jumping straight to it would let the rover step over an obstacle
    it has not seen yet and make the comparison meaningless.
    """
    distance = math.dist(position, subgoal)
    if distance == 0.0:
        return position, 0.0
    ratio = min(1.0, limit / distance)
    moved = distance * ratio
    return (
        position[0] + (subgoal[0] - position[0]) * ratio,
        position[1] + (subgoal[1] - position[1]) * ratio,
    ), moved


def main():
    rows = []
    for name, case in SCENARIOS.items():
        args = (case["obstacles"], case["start"], case["goal"], case["bounds"])
        straight = math.dist(case["start"], case["goal"])
        rows.append((name, straight, run_window(*args), run_global(*args)))

    width = max(len(name) for name, *_ in rows)
    print(f"{'scenario'.ljust(width)}  {'straight':>8}  {'window':>22}  {'global':>22}")
    for name, straight, window, global_ in rows:
        print(
            f"{name.ljust(width)}  {straight:8.1f}  "
            f"{_format(window):>22}  {_format(global_):>22}"
        )

    failed_window = sum(1 for *_, window, _ in rows if not window[0])
    failed_global = sum(1 for *_, global_ in rows if not global_[0])
    print()
    print(f"reached the goal: window {len(rows) - failed_window}/{len(rows)}, "
          f"global {len(rows) - failed_global}/{len(rows)}")
    return 1 if failed_global else 0


def _format(outcome):
    reached, travelled, steps = outcome
    verdict = "reached" if reached else "STUCK"
    return f"{verdict} {travelled:6.1f} m, {steps:3d} plans"


if __name__ == "__main__":
    sys.exit(main())
