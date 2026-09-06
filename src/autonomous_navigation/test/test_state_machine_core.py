import pytest

from autonomous_navigation.state_machine_core import (
    coordinate_target,
    far_target,
    north_west_meters,
    object_target,
    search_target,
    search_waypoints,
    select_target,
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


def test_search_waypoints_expand_and_wrap_angles():
    points = search_waypoints((10, 20), 200, 1, 4)

    assert len(points) == 3
    assert points[0] == (11, 20)
    assert points[1] != points[2]


def test_north_west_conversion_preserves_expected_signs():
    north, west = north_west_meters((42.0, -76.0), (42.00001, -76.00001))

    assert north > 0
    assert west > 0
    assert north_west_meters((42, -76), (42, -76)) == (0, 0)


@pytest.mark.parametrize(
    "goal, object_name, found, search, expected",
    [
        ((20, 0), "coordinate", False, [], (4, 0)),
        ((1, 0), "coordinate", False, [], (1, 0)),
        ((1, 0), "ar1", False, [(3, 0)], (3, 0)),
        ((1, 0), "ar1", True, [], (2, 0)),
    ],
)
def test_select_target_routes_each_mission_mode(
    goal, object_name, found, search, expected
):
    decision = select_target(
        (0, 0), goal, (2, 0), [(4, 0)], object_name, found, search
    )

    assert decision.target == expected
