#!/usr/bin/env python3

import math
from typing import List, Tuple


def global_to_local(
    robot_north: float,
    robot_west: float,
    robot_yaw: float,
    target_north: float,
    target_west: float,
) -> Tuple[float, float]:
    north_delta = target_north - robot_north
    west_delta = target_west - robot_west
    local_forward = math.cos(robot_yaw) * north_delta + math.sin(robot_yaw) * west_delta
    local_left = -math.sin(robot_yaw) * north_delta + math.cos(robot_yaw) * west_delta
    return local_forward, local_left


def visible_in_fov(
    local_forward: float,
    local_left: float,
    fov_deg: float,
    min_distance_m: float,
    max_distance_m: float,
) -> bool:
    if local_forward < min_distance_m or local_forward > max_distance_m:
        return False
    half_angle = math.radians(fov_deg / 2.0)
    lateral_limit = math.tan(half_angle) * max(local_forward, 1e-6)
    return abs(local_left) <= lateral_limit


def expand_box_cluster(
    center_north: float,
    center_west: float,
    size_north: float,
    size_west: float,
    cell_size: float,
    cost_value: float,
) -> List[float]:
    data: List[float] = []
    north_half_steps = max(1, int(math.ceil(size_north / (2.0 * cell_size))))
    west_half_steps = max(1, int(math.ceil(size_west / (2.0 * cell_size))))
    for dx in range(-north_half_steps, north_half_steps + 1):
        for dy in range(-west_half_steps, west_half_steps + 1):
            north = center_north + (dx * cell_size)
            west = center_west + (dy * cell_size)
            north = round(north / cell_size) * cell_size
            west = round(west / cell_size) * cell_size
            data.extend([north, west, cost_value])
    return data
