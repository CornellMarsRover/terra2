import math

import pytest

from autonomous_navigation.global_planner_core import (
    GlobalPlan,
    MapConfig,
    WorldMap,
    astar,
    frontier_subgoal,
    inflate,
    plan,
    quantize,
    reachable_cells,
    route_subgoal,
)


def wall(world, x, y_values, frames=2):
    """Observe a vertical wall often enough for it to be confirmed."""
    for _ in range(frames):
        world.integrate({(x, y): 10.0 for y in y_values})
    return world


def test_quantize_snaps_to_cell_centres():
    assert quantize((1.2, -1.2), 0.5) == (1.0, -1.0)
    assert quantize((0.26, 0.24), 0.5) == (0.5, 0.0)
    with pytest.raises(ValueError):
        quantize((0.0, 0.0), 0.0)


def test_map_config_rejects_impossible_evidence_rules():
    with pytest.raises(ValueError):
        MapConfig(cell_size=0.0)
    with pytest.raises(ValueError):
        MapConfig(hit=0.0)
    with pytest.raises(ValueError):
        MapConfig(miss=-1.0)
    with pytest.raises(ValueError):
        MapConfig(occupied_threshold=5.0, ceiling=1.0)


def test_obstacle_needs_repeated_evidence_before_it_blocks():
    world = WorldMap(MapConfig(cell_size=0.5, occupied_threshold=4.0, hit=3.0))
    world.integrate({(1.0, 0.0): 10.0})
    assert world.confidence((1.0, 0.0)) == 3.0
    assert not world.is_blocked((1.0, 0.0))
    world.integrate({(1.0, 0.0): 10.0})
    assert world.is_blocked((1.0, 0.0))


def test_confidence_is_capped_and_a_single_false_positive_fades():
    world = WorldMap(MapConfig(cell_size=0.5, hit=3.0, miss=1.0, ceiling=6.0))
    for _ in range(5):
        world.integrate({(1.0, 0.0): 10.0})
    assert world.confidence((1.0, 0.0)) == 6.0

    ghost = WorldMap(MapConfig(cell_size=0.5, hit=3.0, miss=1.0))
    ghost.integrate({(2.0, 0.0): 10.0})
    for _ in range(3):
        ghost.integrate({(2.0, 0.0): 0.0})
    assert ghost.confidence((2.0, 0.0)) == 0.0
    assert not ghost.is_blocked((2.0, 0.0))


def test_a_confirmed_wall_survives_leaving_the_camera_view():
    """The failure this module exists for: the local costmap decays, a map must not."""
    world = WorldMap(MapConfig(cell_size=0.5, hit=3.0, miss=1.0))
    wall(world, 3.0, [0.0], frames=4)
    assert world.is_blocked((3.0, 0.0))
    for _ in range(20):
        world.integrate({(9.0, 9.0): 0.0}, observed=[(9.0, 9.0)])
    assert world.is_blocked((3.0, 0.0))


def test_unobserved_cells_are_not_treated_as_cleared():
    world = WorldMap(MapConfig(cell_size=0.5, hit=3.0, miss=1.0))
    wall(world, 3.0, [0.0], frames=2)
    world.integrate({(1.0, 0.0): 0.0}, observed=[(1.0, 0.0)])
    assert world.is_blocked((3.0, 0.0))


def test_driving_through_a_cell_clears_it():
    world = WorldMap(MapConfig(cell_size=0.5))
    wall(world, 1.0, [0.0], frames=4)
    assert world.is_blocked((1.0, 0.0))
    assert world.mark_visited((1.0, 0.0)) == (1.0, 0.0)
    assert not world.is_blocked((1.0, 0.0))
    assert world.visited_cells() == {(1.0, 0.0)}
    assert world.blocked_cells() == set()


def test_bounds_cover_the_map_the_rover_and_the_goal():
    world = WorldMap(MapConfig(cell_size=0.5))
    wall(world, 3.0, [2.0], frames=2)
    world.mark_visited((-1.0, -1.0))
    assert world.bounds((0.0, 0.0), (5.0, 0.0), 1.0) == (-2.0, 6.0, -2.0, 3.0)
    with pytest.raises(ValueError):
        world.bounds((0.0, 0.0), (1.0, 0.0), -1.0)


def test_inflate_grows_obstacles_within_a_circular_radius():
    assert inflate([(0.0, 0.0)], 1.0, 0) == {(0.0, 0.0)}
    grown = inflate([(0.0, 0.0)], 1.0, 1)
    assert (1.0, 0.0) in grown
    assert (1.0, 1.0) not in grown
    assert inflate([], 1.0, 2) == set()
    with pytest.raises(ValueError):
        inflate([(0.0, 0.0)], 0.0, 1)
    with pytest.raises(ValueError):
        inflate([(0.0, 0.0)], 1.0, -1)


def test_astar_finds_a_straight_route_and_validates_inputs():
    route = astar((0.0, 0.0), (2.0, 0.0), set(), 1.0, (-1.0, 3.0, -1.0, 1.0))
    assert route == [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    with pytest.raises(ValueError):
        astar((0.0, 0.0), (1.0, 0.0), set(), 0.0, (0.0, 1.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        astar((0.0, 0.0), (1.0, 0.0), set(), 1.0, (0.0, 1.0, 0.0, 1.0), visited_discount=0.0)
    with pytest.raises(ValueError):
        astar((0.0, 0.0), (1.0, 0.0), set(), 1.0, (0.0, 1.0, 0.0, 1.0), max_expansions=0)


def test_astar_refuses_a_blocked_or_out_of_bounds_goal():
    assert astar((0.0, 0.0), (1.0, 0.0), {(1.0, 0.0)}, 1.0, (-1.0, 2.0, -1.0, 1.0)) == []
    assert astar((0.0, 0.0), (9.0, 0.0), set(), 1.0, (-1.0, 2.0, -1.0, 1.0)) == []


def test_astar_routes_around_a_wall_rather_than_through_it():
    blocked = {(1.0, y) for y in (-1.0, 0.0, 1.0)}
    route = astar((0.0, 0.0), (2.0, 0.0), blocked, 1.0, (-2.0, 3.0, -3.0, 3.0))
    assert route[0] == (0.0, 0.0)
    assert route[-1] == (2.0, 0.0)
    assert not set(route) & blocked


def test_astar_returns_nothing_when_the_goal_is_walled_off():
    blocked = {(1.0, y) for y in (-2.0, -1.0, 0.0, 1.0, 2.0)}
    assert astar((0.0, 0.0), (2.0, 0.0), blocked, 1.0, (-1.0, 3.0, -2.0, 2.0)) == []


def test_astar_gives_up_after_an_expansion_budget():
    assert astar((0.0, 0.0), (30.0, 30.0), set(), 1.0, (-40.0, 40.0, -40.0, 40.0), max_expansions=5) == []


def test_astar_prefers_ground_already_driven_when_it_is_no_longer():
    bounds = (-1.0, 3.0, -2.0, 2.0)
    visited = {(1.0, -1.0), (2.0, -1.0)}
    route = astar((0.0, 0.0), (3.0, 0.0), {(1.0, 0.0)}, 1.0, bounds, visited=visited)
    assert (1.0, -1.0) in route


def test_astar_reaches_a_goal_that_needs_a_better_second_path():
    blocked = {(1.0, 1.0)}
    route = astar((0.0, 0.0), (2.0, 2.0), blocked, 1.0, (-1.0, 3.0, -1.0, 3.0))
    assert route[-1] == (2.0, 2.0)
    assert (1.0, 1.0) not in route


def test_route_subgoal_looks_a_window_ahead():
    route = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (6.0, 0.0)]
    assert route_subgoal(route, 1.5) == (2.0, 0.0)
    assert route_subgoal(route, 100.0) == (6.0, 0.0)
    assert route_subgoal([], 1.0) is None
    with pytest.raises(ValueError):
        route_subgoal(route, 0.0)


def test_reachable_cells_stops_at_walls_and_bounds():
    blocked = {(1.0, y) for y in (-1.0, 0.0, 1.0)}
    reached = reachable_cells((0.0, 0.0), blocked, 1.0, (-1.0, 2.0, -1.0, 1.0))
    assert (2.0, 0.0) not in reached
    assert (0.0, 1.0) in reached
    assert reachable_cells((0.0, 0.0), {(0.0, 0.0)}, 1.0, (-1.0, 1.0, -1.0, 1.0)) == set()
    capped = reachable_cells((0.0, 0.0), set(), 1.0, (-5.0, 5.0, -5.0, 5.0), max_cells=3)
    assert 3 <= len(capped) < 121
    with pytest.raises(ValueError):
        reachable_cells((0.0, 0.0), set(), 0.0, (0.0, 1.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        reachable_cells((0.0, 0.0), set(), 1.0, (0.0, 1.0, 0.0, 1.0), max_cells=0)


def test_frontier_picks_the_reachable_cell_closest_to_the_goal():
    blocked = {(1.0, y) for y in (-2.0, -1.0, 0.0, 1.0, 2.0)}
    frontier = frontier_subgoal((0.0, 0.0), (5.0, 0.0), blocked, 1.0, (-1.0, 3.0, -2.0, 2.0))
    assert frontier == (0.0, -1.0)


def test_frontier_prefers_ground_already_driven_on_a_tie():
    frontier = frontier_subgoal(
        (0.0, 0.0),
        (0.0, 5.0),
        {(0.0, 1.0)},
        1.0,
        (-2.0, 2.0, -2.0, 1.0),
        visited={(1.0, 1.0)},
    )
    assert frontier == (1.0, 1.0)


def test_frontier_reverses_out_of_a_corridor_instead_of_driving_deeper():
    """Isolates why a frontier is not just the reachable cell nearest the goal.

    A dead-end corridor runs from x=0 to x=4 with its far wall at x=5 and its
    mouth at x=-1. The cell nearest the goal is the far wall, four metres the
    wrong way. Only the mouth still touches ground the rover has never seen.
    """
    cell_size = 1.0
    corridor = [(float(x), 0.0) for x in range(5)]
    walls = {(5.0, 0.0)}
    walls |= {(float(x), y) for x in range(-1, 6) for y in (1.0, -1.0)}
    known = set(corridor) | walls

    bounds = (-4.0, 6.0, -1.0, 1.0)
    deeper = frontier_subgoal((2.0, 0.0), (9.0, 0.0), walls, cell_size, bounds)
    assert deeper == (4.0, 0.0)

    out = frontier_subgoal((2.0, 0.0), (9.0, 0.0), walls, cell_size, bounds, known=known)
    assert out == (0.0, 0.0)


def test_frontier_is_none_when_the_rover_cannot_move():
    blocked = {
        (dx, dy)
        for dx in (-1.0, 0.0, 1.0)
        for dy in (-1.0, 0.0, 1.0)
        if (dx, dy) != (0.0, 0.0)
    }
    assert frontier_subgoal((0.0, 0.0), (5.0, 0.0), blocked, 1.0, (-1.0, 1.0, -1.0, 1.0)) is None


def test_plan_hands_the_waypoint_straight_through_on_an_empty_map():
    world = WorldMap(MapConfig(cell_size=0.5))
    result = plan((0.0, 0.0), (10.0, 0.0), world)
    assert result.reason == "direct"
    assert result.subgoal == (10.0, 0.0)
    assert not result.replanned


def test_plan_reports_arrival_without_searching():
    world = WorldMap(MapConfig(cell_size=0.5))
    result = plan((0.0, 0.0), (0.5, 0.0), world, arrival=1.0)
    assert result == GlobalPlan((0.5, 0.0), ((0.5, 0.0),), "direct")
    with pytest.raises(ValueError):
        plan((0.0, 0.0), (5.0, 0.0), world, arrival=0.0)


def test_plan_routes_around_a_wall_it_remembers():
    world = WorldMap(MapConfig(cell_size=0.5))
    wall(world, 5.0, [i * 0.5 - 3.0 for i in range(13)])
    result = plan((0.0, 0.0), (10.0, 0.0), world)
    assert result.reason == "routed"
    assert result.replanned
    assert abs(result.subgoal[1]) > 1.0


def test_plan_drives_at_a_frontier_when_the_goal_is_walled_off():
    world = WorldMap(MapConfig(cell_size=1.0))
    ring = [
        (5.0 + dx, 0.0 + dy)
        for dx in (-1.0, 0.0, 1.0)
        for dy in (-1.0, 0.0, 1.0)
        if (dx, dy) != (0.0, 0.0)
    ]
    for _ in range(2):
        world.integrate({cell: 10.0 for cell in ring})
    result = plan((0.0, 0.0), (5.0, 0.0), world, inflation_radius=0)
    assert result.reason == "frontier"
    assert result.replanned


def test_plan_reports_trapped_when_there_is_no_move_at_all():
    world = WorldMap(MapConfig(cell_size=1.0))
    ring = [
        (dx, dy)
        for dx in (-1.0, 0.0, 1.0)
        for dy in (-1.0, 0.0, 1.0)
        if (dx, dy) != (0.0, 0.0)
    ]
    for _ in range(2):
        world.integrate({cell: 10.0 for cell in ring})
    result = plan((0.0, 0.0), (5.0, 0.0), world, inflation_radius=0, margin=0.0)
    assert result.reason == "trapped"
    assert result.subgoal == (0.0, 0.0)


def test_plan_calls_any_searched_route_routed_even_when_it_ends_at_the_goal():
    world = WorldMap(MapConfig(cell_size=1.0))
    for _ in range(2):
        world.integrate({(0.0, 3.0): 10.0})
    short = plan((0.0, 0.0), (2.0, 0.0), world, lookahead=0.5, inflation_radius=0)
    assert short.reason == "routed"
    long = plan((0.0, 0.0), (2.0, 0.0), world, lookahead=50.0, inflation_radius=0)
    assert long.reason == "routed"
    assert long.subgoal == (2.0, 0.0)


def test_plan_ignores_an_obstacle_it_has_already_driven_through():
    world = WorldMap(MapConfig(cell_size=1.0))
    for _ in range(2):
        world.integrate({(1.0, 0.0): 10.0})
    world.mark_visited((1.0, 0.0))
    result = plan((0.0, 0.0), (5.0, 0.0), world)
    assert result.reason == "direct"


def test_maze_is_solved_end_to_end_and_a_window_planner_is_not():
    """The regression that proves the point. Numbers go in the CDR deck."""
    walls = set()
    for y in range(-4, 3):
        walls.add((4.0, float(y)))
    for y in range(-2, 5):
        walls.add((8.0, float(y)))
    bounds = (-2.0, 14.0, -6.0, 6.0)
    goal = (12.0, 0.0)

    world = WorldMap(MapConfig(cell_size=1.0))
    for _ in range(2):
        world.integrate({cell: 10.0 for cell in walls})

    position = (0.0, 0.0)
    for _ in range(80):
        world.mark_visited(position)
        result = plan(position, goal, world, lookahead=2.0, inflation_radius=0, margin=2.0)
        if math.dist(position, goal) <= 1.0:
            break
        position = result.subgoal
    assert math.dist(position, goal) <= 1.0

    # A window-only planner clamps the goal into its own box and stalls on the wall.
    window = 3.0
    position = (0.0, 0.0)
    for _ in range(80):
        clamped = (
            max(min(goal[0], position[0] + window), position[0] - window),
            max(min(goal[1], position[1] + window), position[1] - window),
        )
        step = astar(
            position,
            clamped,
            walls,
            1.0,
            (position[0] - window, position[0] + window, position[1] - window, position[1] + window),
        )
        if not step or len(step) < 2:
            break
        position = step[1]
    assert math.dist(position, goal) > 1.0
