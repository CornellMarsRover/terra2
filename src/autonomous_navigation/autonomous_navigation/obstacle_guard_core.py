#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Sequence, Tuple


CostEntry = Tuple[float, float, float]


@dataclass
class ObstacleSummary:
    blocked: bool
    nearest_distance: float
    left_cost: float
    right_cost: float
    center_cost: float


def global_to_local(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return local_x, local_y


def summarize_front_obstacles(
    costs: Sequence[CostEntry],
    robot_north: float,
    robot_west: float,
    yaw: float,
    lookahead_distance: float,
    corridor_half_width: float,
    cost_threshold: float,
) -> ObstacleSummary:
    nearest_distance = math.inf
    left_cost = 0.0
    right_cost = 0.0
    center_cost = 0.0

    for north, west, cost in costs:
        if cost < cost_threshold:
            continue
        local_x, local_y = global_to_local(north - robot_north, west - robot_west, yaw)
        if local_x <= 0.0 or local_x > lookahead_distance or abs(local_y) > corridor_half_width:
            continue

        nearest_distance = min(nearest_distance, local_x)
        if local_y > 0.1:
            left_cost += cost
        elif local_y < -0.1:
            right_cost += cost
        else:
            center_cost += cost

    blocked = math.isfinite(nearest_distance)
    return ObstacleSummary(
        blocked=blocked,
        nearest_distance=nearest_distance,
        left_cost=left_cost,
        right_cost=right_cost,
        center_cost=center_cost,
    )


def choose_turn_direction(summary: ObstacleSummary) -> float:
    if not summary.blocked:
        return 0.0
    if summary.left_cost > summary.right_cost:
        return -1.0
    if summary.right_cost > summary.left_cost:
        return 1.0
    return 1.0
