import math

from formation_interface.path_planner import GridAStarPlanner
import pytest

BOUNDS = (0.0, 10.0, 0.0, 10.0)


def _path_length(path):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])
    )


def test_no_obstacles_gives_a_near_direct_path():
    planner = GridAStarPlanner(BOUNDS, cell_size=0.25, safety_radius=0.5)
    start, goal = (1.0, 1.0), (9.0, 9.0)
    path = planner.plan(start, goal, obstacles=[])
    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
    # grid discretization adds a little slack, but shouldn't meaningfully detour.
    assert _path_length(path) < straight * 1.15


def test_single_active_drone_case_is_obstacle_free():
    # obstacles=() is exactly what the caller passes when only one drone is active.
    planner = GridAStarPlanner(BOUNDS, cell_size=0.5, safety_radius=0.5)
    path = planner.plan((0.5, 0.5), (9.5, 0.5), obstacles=())
    assert path is not None
    assert path[-1] == (9.5, 0.5)


def test_path_routes_around_a_blocking_obstacle():
    planner = GridAStarPlanner(BOUNDS, cell_size=0.25, safety_radius=1.0)
    start, goal = (1.0, 5.0), (9.0, 5.0)
    obstacle = (5.0, 5.0)   # sits directly on the straight line start->goal
    path = planner.plan(start, goal, obstacles=[obstacle])
    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    for x, y in path:
        assert math.hypot(x - obstacle[0], y - obstacle[1]) >= planner.cell_size


def test_goal_boxed_in_returns_none():
    planner = GridAStarPlanner(BOUNDS, cell_size=0.5, safety_radius=0.6)
    goal = (5.0, 5.0)
    # ring of obstacles tight enough (with inflation) to seal the goal cell off.
    obstacles = [
        (goal[0] + dx, goal[1] + dy)
        for dx in (-0.5, 0.0, 0.5)
        for dy in (-0.5, 0.0, 0.5)
        if (dx, dy) != (0.0, 0.0)
    ]
    path = planner.plan((1.0, 1.0), goal, obstacles=obstacles)
    assert path is None


def test_start_outside_bounds_raises():
    planner = GridAStarPlanner(BOUNDS)
    with pytest.raises(ValueError):
        planner.plan((-1.0, -1.0), (5.0, 5.0), obstacles=[])


def test_goal_outside_bounds_raises():
    planner = GridAStarPlanner(BOUNDS)
    with pytest.raises(ValueError):
        planner.plan((5.0, 5.0), (50.0, 50.0), obstacles=[])


def test_invalid_bounds_rejected():
    with pytest.raises(ValueError):
        GridAStarPlanner((5.0, 1.0, 0.0, 10.0))
