import math

import pytest

from autonomous_navigation.costmap_core import GridObservation, project_point

IDENTITY = ((1.0, 0.0), (0.0, 1.0))


def test_real_point_projects_into_rotated_global_grid():
    rotation = ((0.0, -1.0), (1.0, 0.0))

    result = project_point((2.0, 0.5, -0.8), (10, 20), rotation, True, 1, 1.2, 6, 0.25)

    assert result == GridObservation((9.5, 22.0), 0.2)


@pytest.mark.parametrize(
    "point",
    [
        (0.5, 0.0, 0.0),
        (7.0, 0.0, 0.0),
        (2.0, 3.0, 0.0),
        (-2.0, 0.0, 0.0),
    ],
)
def test_real_point_rejects_invalid_depth_or_bearing(point):
    assert project_point(point, (0, 0), IDENTITY, True, 1, 1.2, 6, 0.25) is None


def test_sim_point_uses_camera_axes_and_flips_global_left():
    result = project_point((0.6, 0.8, 2.0), (1, 3), IDENTITY, False, 1, 1.2, 6, 0.25)

    assert result == GridObservation((3.0, 2.5), 0.2)


@pytest.mark.parametrize("depth", [0.5, 7.0])
def test_sim_point_rejects_invalid_depth(depth):
    point = (0.0, 0.0, depth)

    assert project_point(point, (0, 0), IDENTITY, False, 1, 1.2, 6, 0.25) is None


def test_projection_snaps_to_requested_cell_size():
    angle = math.radians(0)
    rotation = ((math.cos(angle), -math.sin(angle)), (math.sin(angle), math.cos(angle)))

    result = project_point((1.26, 0, 2.24), (0, 0), rotation, False, 1, 1, 6, 0.5)

    assert result.cell == (2.0, -1.5)


from autonomous_navigation.costmap_core import decay_costs, observed_cost  # noqa: E402


def test_observed_cost_marks_ground_and_overhead_as_traversable():
    assert observed_cost(4, 0.0, -0.2, 0.15, 2, 100) is None
    assert observed_cost(4, 3.0, -0.2, 0.15, 2, 100) is None


def test_observed_cost_accumulates_low_and_tall_obstacles_with_cap():
    assert observed_cost(4, 0.3, -0.2, 0.15, 2, 100) == 6
    assert observed_cost(97, 0.8, -0.2, 0.15, 2, 100) == 100


def test_decay_costs_removes_zeroes_and_preserves_ineligible_cells():
    costs = {(0, 0): 1, (1, 0): 4, (2, 0): 5}

    assert decay_costs(costs, 2, eligible={(0, 0), (1, 0)}) == {
        (1, 0): 2,
        (2, 0): 5,
    }
    assert costs == {(0, 0): 1, (1, 0): 4, (2, 0): 5}


def test_decay_costs_defaults_to_all_cells():
    assert decay_costs({(0, 0): 2}, 1) == {(0, 0): 1}


def test_decay_costs_rejects_negative_amount():
    with pytest.raises(ValueError, match="negative"):
        decay_costs({}, -1)
