import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autonomous_navigation.obstacle_guard_core import (
    choose_turn_direction,
    summarize_front_obstacles,
)


def test_front_obstacle_is_detected_inside_corridor():
    summary = summarize_front_obstacles(
        costs=[(1.0, 0.0, 10.0), (2.5, 2.5, 15.0)],
        robot_north=0.0,
        robot_west=0.0,
        yaw=0.0,
        lookahead_distance=1.5,
        corridor_half_width=0.5,
        cost_threshold=8.0,
    )

    assert summary.blocked is True
    assert math.isclose(summary.nearest_distance, 1.0)
    assert summary.center_cost == 10.0


def test_turns_away_from_left_heavy_obstacle_cluster():
    summary = summarize_front_obstacles(
        costs=[(1.0, 0.3, 12.0), (1.2, 0.35, 12.0), (1.0, -0.2, 6.0)],
        robot_north=0.0,
        robot_west=0.0,
        yaw=0.0,
        lookahead_distance=1.5,
        corridor_half_width=0.6,
        cost_threshold=5.0,
    )

    assert choose_turn_direction(summary) < 0.0


def test_respects_robot_yaw_when_projecting_costs():
    summary = summarize_front_obstacles(
        costs=[(0.0, 1.0, 10.0)],
        robot_north=0.0,
        robot_west=0.0,
        yaw=math.pi / 2.0,
        lookahead_distance=1.5,
        corridor_half_width=0.5,
        cost_threshold=8.0,
    )

    assert summary.blocked is True
    assert math.isclose(summary.nearest_distance, 1.0, rel_tol=1e-5)
