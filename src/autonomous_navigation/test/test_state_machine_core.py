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
    assert coordinate_target((0, 0), (1, 0)).reached
    assert not coordinate_target((0, 0), (3, 0)).reached


def test_search_target_reports_empty_search_as_timeout():
    decision = search_target((0, 0), (4, 4), [])

    assert decision.target == (4, 4)
    assert decision.timed_out


def test_search_target_announces_active_search():
    decision = search_target((0, 0), (4, 4), [(3, 0)])

    assert decision.target == (3, 0)
    assert decision.announce_object


def test_search_target_times_out_after_final_reached_point():
    decision = search_target((1, 0), (4, 4), [(1, 0)])

    assert decision.target == (4, 4)
    assert decision.timed_out
    assert decision.pop_search


def test_search_target_advances_to_following_point():
    decision = search_target((1, 0), (4, 4), [(1, 0), (2, 0)])

    assert decision.target == (2, 0)
    assert decision.pop_search
    assert not decision.announce_object


def test_object_target_reports_standoff_crossing():
    assert object_target((0, 0), (1, 0)).reached
    assert not object_target((0, 0), (2, 0)).reached
