#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np


def load_samples(path: Path) -> List[Dict]:
    payload = json.loads(path.read_text())
    return payload.get("samples", [])


def iter_points(sample: Dict) -> Iterable[Tuple[float, float]]:
    pose = sample.get("pose")
    if pose:
        yield pose["west"], pose["north"]

    for key in ("goal", "local_target", "next_waypoint"):
        value = sample.get(key)
        if value and len(value) >= 2:
            yield value[1], value[0]

    costmap = sample.get("costmap") or []
    for idx in range(0, len(costmap), 3):
        if idx + 1 < len(costmap):
            yield costmap[idx + 1], costmap[idx]

    for key in ("all_obstacles", "visible_obstacles"):
        obstacle_data = sample.get(key) or []
        for idx in range(0, len(obstacle_data), 4):
            if idx + 1 < len(obstacle_data):
                yield obstacle_data[idx + 1], obstacle_data[idx]


def compute_bounds(samples: List[Dict], padding: float = 0.8) -> Tuple[float, float, float, float]:
    west_values: List[float] = []
    north_values: List[float] = []
    for sample in samples:
        for west, north in iter_points(sample):
            west_values.append(west)
            north_values.append(north)

    if not west_values:
        return -2.0, 2.0, -2.0, 8.0

    west_min = min(west_values) - padding
    west_max = max(west_values) + padding
    north_min = min(north_values) - padding
    north_max = max(north_values) + padding

    span = max(west_max - west_min, north_max - north_min)
    west_center = (west_min + west_max) / 2.0
    north_center = (north_min + north_max) / 2.0
    half = span / 2.0
    return west_center - half, west_center + half, north_center - half, north_center + half


def world_to_px(
    west: float,
    north: float,
    west_min: float,
    west_max: float,
    north_min: float,
    north_max: float,
    size: int,
    margin: int,
) -> Tuple[int, int]:
    drawable = size - (2 * margin)
    west_norm = (west - west_min) / max(west_max - west_min, 1e-6)
    north_norm = (north - north_min) / max(north_max - north_min, 1e-6)
    x = int(margin + west_norm * drawable)
    y = int(size - margin - north_norm * drawable)
    return x, y


def draw_label(frame: np.ndarray, text: str, origin: Tuple[int, int], color: Tuple[int, int, int]) -> None:
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)


def draw_robot(
    frame: np.ndarray,
    west: float,
    north: float,
    yaw: float,
    bounds: Tuple[float, float, float, float],
    size: int,
    margin: int,
) -> None:
    center = np.array(world_to_px(west, north, *bounds, size, margin), dtype=np.float32)
    scale = 18.0
    heading = np.array([math.sin(yaw), math.cos(yaw)], dtype=np.float32)
    left = np.array([math.sin(yaw + 2.4), math.cos(yaw + 2.4)], dtype=np.float32)
    right = np.array([math.sin(yaw - 2.4), math.cos(yaw - 2.4)], dtype=np.float32)

    points = np.array(
        [
            center + np.array([heading[0], -heading[1]]) * scale,
            center + np.array([left[0], -left[1]]) * scale * 0.8,
            center + np.array([right[0], -right[1]]) * scale * 0.8,
        ],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(frame, points, (255, 200, 40))


def draw_fov(
    frame: np.ndarray,
    west: float,
    north: float,
    yaw: float,
    bounds: Tuple[float, float, float, float],
    size: int,
    margin: int,
    range_m: float = 3.5,
    fov_deg: float = 70.0,
) -> None:
    center = world_to_px(west, north, *bounds, size, margin)
    half = math.radians(fov_deg / 2.0)
    endpoints = []
    for angle in (yaw - half, yaw + half):
        end_west = west + math.sin(angle) * range_m
        end_north = north + math.cos(angle) * range_m
        endpoints.append(world_to_px(end_west, end_north, *bounds, size, margin))

    overlay = frame.copy()
    polygon = np.array([center, endpoints[0], endpoints[1]], dtype=np.int32)
    cv2.fillConvexPoly(overlay, polygon, (60, 120, 60))
    cv2.addWeighted(overlay, 0.18, frame, 0.82, 0, frame)


def draw_costmap(
    frame: np.ndarray,
    costmap: List[float],
    bounds: Tuple[float, float, float, float],
    size: int,
    margin: int,
) -> None:
    if not costmap:
        return

    values = costmap[2::3]
    max_cost = max(values) if values else 100.0
    for idx in range(0, len(costmap), 3):
        if idx + 2 >= len(costmap):
            break
        north, west, cost = costmap[idx], costmap[idx + 1], costmap[idx + 2]
        x, y = world_to_px(west, north, *bounds, size, margin)
        intensity = min(1.0, cost / max(max_cost, 1e-6))
        color = (
            int(40 + 120 * (1.0 - intensity)),
            int(40 + 80 * (1.0 - intensity)),
            int(140 + 115 * intensity),
        )
        cv2.rectangle(frame, (x - 6, y - 6), (x + 6, y + 6), color, -1)
        cv2.rectangle(frame, (x - 6, y - 6), (x + 6, y + 6), (30, 30, 30), 1)


def draw_obstacle_boxes(
    frame: np.ndarray,
    obstacle_data: List[float],
    bounds: Tuple[float, float, float, float],
    size: int,
    margin: int,
    color: Tuple[int, int, int],
    thickness: int,
) -> None:
    for idx in range(0, len(obstacle_data), 4):
        if idx + 3 >= len(obstacle_data):
            break
        north, west, size_north, size_west = obstacle_data[idx:idx + 4]
        top_left = world_to_px(
            west - (size_west / 2.0),
            north + (size_north / 2.0),
            *bounds,
            size,
            margin,
        )
        bottom_right = world_to_px(
            west + (size_west / 2.0),
            north - (size_north / 2.0),
            *bounds,
            size,
            margin,
        )
        cv2.rectangle(frame, top_left, bottom_right, color, thickness)


def render(
    samples: List[Dict],
    output: Path,
    fps: float,
    size: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output), fourcc, fps, (size, size))
    bounds = compute_bounds(samples)
    margin = 64

    trail: List[Tuple[int, int]] = []
    for sample in samples:
        frame = np.full((size, size, 3), 242, dtype=np.uint8)

        for grid in np.linspace(0.1, 0.9, 5):
            offset = int(margin + grid * (size - 2 * margin))
            cv2.line(frame, (margin, offset), (size - margin, offset), (215, 215, 215), 1)
            cv2.line(frame, (offset, margin), (offset, size - margin), (215, 215, 215), 1)

        cv2.rectangle(frame, (margin, margin), (size - margin, size - margin), (120, 120, 120), 2)

        pose = sample.get("pose")
        goal = sample.get("goal")
        local_target = sample.get("local_target")
        next_waypoint = sample.get("next_waypoint")
        costmap = sample.get("costmap") or []
        active = bool(sample.get("avoidance_active"))
        all_obstacles = sample.get("all_obstacles") or []
        visible_obstacles = sample.get("visible_obstacles") or []
        t = float(sample.get("t", 0.0))

        draw_obstacle_boxes(frame, all_obstacles, bounds, size, margin, (140, 140, 140), 2)
        draw_costmap(frame, costmap, bounds, size, margin)
        draw_obstacle_boxes(frame, visible_obstacles, bounds, size, margin, (0, 40, 220), 3)

        if goal and len(goal) >= 2:
            gx, gy = world_to_px(goal[1], goal[0], *bounds, size, margin)
            cv2.drawMarker(frame, (gx, gy), (40, 160, 40), cv2.MARKER_TILTED_CROSS, 22, 3)
            draw_label(frame, "goal", (gx + 10, gy - 10), (30, 90, 30))

        if local_target and len(local_target) >= 2:
            lx, ly = world_to_px(local_target[1], local_target[0], *bounds, size, margin)
            cv2.circle(frame, (lx, ly), 8, (210, 150, 50), -1)
            draw_label(frame, "local target", (lx + 10, ly + 18), (125, 80, 20))

        if next_waypoint and len(next_waypoint) >= 2:
            wx, wy = world_to_px(next_waypoint[1], next_waypoint[0], *bounds, size, margin)
            cv2.circle(frame, (wx, wy), 8, (190, 60, 200), -1)
            draw_label(frame, "next waypoint", (wx + 10, wy - 10), (110, 30, 120))

        if pose:
            px, py = world_to_px(pose["west"], pose["north"], *bounds, size, margin)
            trail.append((px, py))
            if len(trail) > 1:
                cv2.polylines(frame, [np.array(trail, dtype=np.int32)], False, (220, 110, 70), 3)
            draw_fov(frame, pose["west"], pose["north"], pose["yaw"], bounds, size, margin)
            draw_robot(frame, pose["west"], pose["north"], pose["yaw"], bounds, size, margin)

        draw_label(frame, "Autonomy obstacle avoidance", (24, 36), (40, 40, 40))
        draw_label(frame, f"t = {t:4.1f}s", (24, 68), (50, 50, 50))
        draw_label(
            frame,
            f"avoidance {'ACTIVE' if active else 'tracking'}",
            (24, 100),
            (25, 40, 180) if active else (70, 70, 70),
        )
        draw_label(frame, f"visible obstacles: {len(visible_obstacles) // 4}", (24, 132), (50, 50, 50))
        draw_label(frame, "gray boxes = scenario obstacles", (24, size - 56), (60, 60, 60))
        draw_label(frame, "blue boxes/red cells = detected obstacles", (24, size - 28), (60, 60, 60))

        writer.write(frame)

    writer.release()


def main() -> None:
    parser = argparse.ArgumentParser(description="Render a top-down autonomy demo map video.")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--size", type=int, default=900)
    args = parser.parse_args()

    samples = load_samples(args.input)
    render(samples=samples, output=args.output, fps=args.fps, size=args.size)


if __name__ == "__main__":
    main()
