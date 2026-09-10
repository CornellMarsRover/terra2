import math

import pytest

from autonomous_navigation.drive_command import (
    ChassisCommand,
    forward_heading_command,
    inputs_fresh,
    point_turn_command,
    waypoint_reached,
)


def test_forward_command_uses_normalized_chassis_axes():
    command = forward_heading_command(0.6, 20.0)

    assert command.linear_x == pytest.approx(0.6)
    assert command.linear_y == 0.0
    assert command.angular_z == pytest.approx(math.radians(20.0))


@pytest.mark.parametrize("position", [(0.0, 0.0), (0.3, 0.0)])
def test_waypoint_reached_includes_tolerance_boundary(position):
    assert waypoint_reached(position, (0.0, 0.0))


def test_waypoint_reached_rejects_outside_and_invalid_tolerance():
    assert not waypoint_reached((0.31, 0.0), (0.0, 0.0))
    with pytest.raises(ValueError, match="negative"):
        waypoint_reached((0.0, 0.0), (0.0, 0.0), -0.1)


def test_forward_command_clamps_speed_and_heading():
    command = forward_heading_command(4.0, 90.0)

    assert command.linear_x == 1.0
    assert command.angular_z == pytest.approx(math.radians(35.0))


@pytest.mark.parametrize("angular_z", [0.4, -0.4, 0.0])
def test_point_turn_preserves_ros_yaw_sign(angular_z):
    assert point_turn_command(angular_z).angular_z == angular_z


def test_point_turn_clamps_to_normalized_range():
    command = point_turn_command(3.0)
    assert command.angular_z == 1.0


def test_nonpositive_heading_bound_is_rejected():
    with pytest.raises(ValueError):
        forward_heading_command(0.5, 10.0, max_heading_error_deg=0.0)


def test_zero_command_is_all_zero():
    assert ChassisCommand() == ChassisCommand(0.0, 0.0, 0.0)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_drive_commands_reject_nonfinite_values(value):
    with pytest.raises(ValueError, match="finite"):
        point_turn_command(value)


@pytest.mark.parametrize(
    "now,pose,waypoint,timeout,expected",
    [
        (10, 10, 10, 1, True), (10, 9, 9, 1, True),
        (10, 8.99, 10, 1, False), (10, 10, 8.99, 1, False),
        (10, None, 10, 1, False), (10, 10, None, 1, False),
        (10, 10.01, 10, 1, False), (10, 10, 10.01, 1, False),
        (10, math.nan, 10, 1, False), (10, 10, math.inf, 1, False),
    ],
)
def test_input_freshness_matrix(now, pose, waypoint, timeout, expected):
    assert inputs_fresh(now, pose, waypoint, timeout) is expected


@pytest.mark.parametrize("timeout", [0, -1, math.inf, math.nan])
def test_input_freshness_rejects_invalid_timeout(timeout):
    with pytest.raises(ValueError, match="timeout"):
        inputs_fresh(1, 1, 1, timeout)
