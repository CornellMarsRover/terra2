import pytest

from autonomous_navigation.planner_core import (
    advance_path,
    neighbor_cost,
    nearest_clear_goal,
    parse_costmap,
    path_is_dense,
    perpendicular_distance,
    record_segment_observation,
    segment_cost,
    simplify_path,
)


def test_segment_observations_require_consecutive_confirmations():
    original = {}
    counts, confirmed = record_segment_observation(original, (0, 1), True)
    assert original == {}
    assert counts == {(0, 1): 1}
    assert not confirmed

    counts, confirmed = record_segment_observation(counts, (0, 1), True, 2)
    assert confirmed
    counts, confirmed = record_segment_observation(counts, (0, 1), False, 2)
    assert counts == {}
    assert not confirmed


def test_segment_observations_reject_invalid_confirmation_count():
    with pytest.raises(ValueError, match="positive"):
        record_segment_observation({}, (0, 1), True, 0)


def test_nearest_clear_goal_clamps_and_accepts_clear_goal():
    result = nearest_clear_goal((4, -1), (0, 2, 0, 2), lambda point: False, 1)
    assert result == (2, 0)


def test_nearest_clear_goal_uses_deterministic_neighbor():
    result = nearest_clear_goal(
        (1, 1),
        (0, 2, 0, 2),
        lambda point: point == (1, 1),
        1,
    )
    assert result == (0, 1)


def test_nearest_clear_goal_is_bounded_when_every_cell_is_blocked():
    result = nearest_clear_goal((1, 1), (0, 2, 0, 2), lambda point: True, 1)
    assert result == (1, 1)


def test_nearest_clear_goal_rejects_nonpositive_step():
    with pytest.raises(ValueError, match="positive"):
        nearest_clear_goal((0, 0), (0, 1, 0, 1), lambda point: False, 0)


def test_parse_costmap_splits_costs_and_obstacles():
    costs, obstacles = parse_costmap([1, 2, 3, 4, 5, 8], threshold=4)

    assert costs == {(1.0, 2.0): 3.0, (4.0, 5.0): 8.0}
    assert obstacles == {(4.0, 5.0)}


def test_parse_costmap_rejects_incomplete_triple():
    with pytest.raises(ValueError, match="triples"):
        parse_costmap([1, 2], threshold=4)


def test_advance_path_waits_outside_threshold():
    result = advance_path((0, 0), (4, 0), [(2, 0), (4, 0)], (2, 0), 0.3)

    assert result == ([(2, 0), (4, 0)], (2, 0), None)


def test_advance_path_selects_following_waypoint():
    result = advance_path((1.9, 0), (4, 0), [(2, 0), (4, 0)], (2, 0), 0.3)

    assert result == ([(4, 0)], (4, 0), (2, 0))


def test_advance_path_falls_back_to_target():
    result = advance_path((2, 0), (4, 0), [], (2, 0), 0.3)

    assert result == ([], (4, 0), (2, 0))


def test_path_density_requires_two_short_segments():
    assert path_is_dense([(0, 0), (0.5, 0), (1.0, 0)])
    assert not path_is_dense([(0, 0), (2, 0), (2.5, 0)])
    assert not path_is_dense([(0, 0), (0.5, 0)])


def test_perpendicular_distance_handles_line_and_point():
    assert perpendicular_distance((1, 1), (0, 0), (2, 0)) == pytest.approx(1)
    assert perpendicular_distance((1, 1), (0, 0), (0, 0)) == pytest.approx(2**0.5)


def test_simplify_path_handles_short_straight_and_bent_paths():
    assert simplify_path([(0, 0), (1, 0)], 0.1) == [(0, 0), (1, 0)]
    assert simplify_path([(0, 0), (1, 0), (2, 0)], 0.1) == [(0, 0), (2, 0)]
    assert simplify_path([(0, 0), (1, 1), (2, 0)], 0.1) == [
        (0, 0),
        (1, 1),
        (2, 0),
    ]


def test_neighbor_cost_weights_cells_and_ignores_unknown_marker():
    costs = {(-0.25, -0.25): 2, (-0.25, 0): 1}

    assert neighbor_cost(costs, (0, 0), 0.25, 1) == pytest.approx(2 / (0.125**0.5))
    assert neighbor_cost({}, (0, 0), 0.25, 0) == 0


def test_segment_cost_handles_stationary_and_sampled_segments():
    assert segment_cost({}, (0, 0), (0, 0)) == (0, 0)

    maximum, total = segment_cost(
        {(0.5, 0.0): 4.0},
        (0, 0),
        (1, 0),
        neighbor_radius=0,
    )
    assert maximum == 4.0
    assert total == 4.0
