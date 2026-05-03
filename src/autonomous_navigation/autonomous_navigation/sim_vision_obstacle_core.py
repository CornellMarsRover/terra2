#!/usr/bin/env python3

import math
from typing import List, Sequence, Tuple


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


def obstacle_angular_span(
    local_forward: float,
    local_left: float,
    size_north: float,
    size_west: float,
) -> Tuple[float, float]:
    half_depth = size_north / 2.0
    half_width = size_west / 2.0
    nearest_forward = max(1e-3, local_forward - half_depth)
    left_min = local_left - half_width
    left_max = local_left + half_width
    angle_min = math.atan2(left_min, nearest_forward)
    angle_max = math.atan2(left_max, nearest_forward)
    return min(angle_min, angle_max), max(angle_min, angle_max)


def intervals_overlap(a: Tuple[float, float], b: Tuple[float, float], min_overlap_rad: float) -> bool:
    overlap = min(a[1], b[1]) - max(a[0], b[0])
    return overlap > min_overlap_rad


def filter_visible_obstacles(
    candidates: Sequence[Tuple[float, float, float, float, float, float]],
    occlusion_overlap_deg: float = 8.0,
) -> List[Tuple[float, float, float, float, float, float]]:
    accepted: List[Tuple[float, float, float, float, float, float]] = []
    covered_spans: List[Tuple[float, float]] = []
    min_overlap = math.radians(occlusion_overlap_deg)

    for candidate in sorted(candidates, key=lambda item: item[4]):
        span = obstacle_angular_span(candidate[4], candidate[5], candidate[2], candidate[3])
        if any(intervals_overlap(span, covered, min_overlap) for covered in covered_spans):
            continue
        accepted.append(candidate)
        covered_spans.append(span)

    return accepted
