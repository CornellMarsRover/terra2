import math

import pytest

from autonomous_navigation.drive_command import (
    ChassisCommand,
    forward_heading_command,
    point_turn_command,
)


def test_forward_command_uses_normalized_chassis_axes():
    command = forward_heading_command(0.6, 20.0)

    assert command.linear_x == pytest.approx(0.6)
    assert command.linear_y == 0.0
    assert command.angular_z == pytest.approx(math.radians(20.0))


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
