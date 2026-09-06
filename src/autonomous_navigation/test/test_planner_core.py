import pytest

from autonomous_navigation.planner_core import (
    advance_path,
    parse_costmap,
    path_is_dense,
)


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
