import pytest

from autonomous_navigation.state_machine_core import (
    coordinate_target,
    far_target,
    object_target,
    search_target,
)


def test_far_target_keeps_detected_object_target():
    decision = far_target((0, 0), (20, 0), (3, 4), [(5, 0)], True)

    assert decision.target == (3, 4)
    assert not decision.pop_coarse


def test_far_target_uses_current_coarse_waypoint():
    decision = far_target((0, 0), (20, 0), (0, 0), [(5, 0)], False)

    assert decision.target == (5, 0)
    assert not decision.pop_coarse


@pytest.mark.parametrize(
    "coarse, expected",
    [
        ([(1, 0), (5, 0)], (5, 0)),
        ([(1, 0)], (20, 0)),
    ],
)
def test_far_target_advances_reached_coarse_waypoint(coarse, expected):
    decision = far_target((0, 0), (20, 0), (0, 0), coarse, False)

    assert decision.target == expected
    assert decision.pop_coarse


def test_coordinate_target_reports_threshold_crossing():
