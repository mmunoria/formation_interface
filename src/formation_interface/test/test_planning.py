import math

import pytest

from formation_interface.planning import Arena, PlanningError, plan_paths

ARENA = Arena(-5.0, 5.0, -5.0, 5.0)


def _min_dist_to(path, point):
    """Smallest distance from any densely-sampled path point to `point`."""
    best = math.inf
    for a, b in zip(path, path[1:]):
        steps = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.02))
        for s in range(steps + 1):
            t = s / steps
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            best = min(best, math.hypot(x - point[0], y - point[1]))
    return best


def test_single_drone_clear_straight_shot():
    paths, _ = plan_paths([(-4.0, 0.0)], [(4.0, 0.0)], ARENA)
    assert len(paths) == 1
    p = paths[0]
    assert p[0] == (-4.0, 0.0)
    assert p[-1] == (4.0, 0.0)
    assert len(p) == 2          # nothing in the way -> simplified to a segment


def test_path_avoids_parked_drone():
    # drone 1 parked at the origin; drone 2 must cross and route around it
    paths, _ = plan_paths([(0.0, 0.0), (-4.0, 0.0)],
                       [(0.0, 0.0), (4.0, 0.0)], ARENA)
    clearance = _min_dist_to(paths[1], (0.0, 0.0))
    # inflated radius 0.4 minus one cell of discretisation slack
    assert clearance > 0.28, f"clearance {clearance:.3f} m"
    assert paths[1][0] == (-4.0, 0.0)
    assert paths[1][-1] == (4.0, 0.0)


def test_avoids_static_obstacle():
    # only drone 2 is being planned for; drone 1's parked position is passed
    # as a static obstacle (e.g. it's holding a target from an earlier,
    # separate goal) rather than as a second start/goal pair.
    paths, _ = plan_paths([(-4.0, 0.0)], [(4.0, 0.0)], ARENA,
                          static_obstacles=[(0.0, 0.0)])
    clearance = _min_dist_to(paths[0], (0.0, 0.0))
    assert clearance > 0.28, f"clearance {clearance:.3f} m"
    assert paths[0][0] == (-4.0, 0.0)
    assert paths[0][-1] == (4.0, 0.0)


def test_all_waypoints_inside_arena():
    starts = [(-4.5, -4.5), (4.5, -4.5), (0.0, 4.5)]
    goals = [(4.5, 4.5), (-4.5, 4.5), (0.0, -4.5)]
    for path in plan_paths(starts, goals, ARENA)[0]:
        for (x, y) in path:
            assert ARENA.contains(x, y)


def test_goal_outside_arena_raises_with_drone_id():
    with pytest.raises(PlanningError, match="drone 1"):
        plan_paths([(0.0, 0.0)], [(9.0, 0.0)], ARENA)


def test_goal_inside_safety_radius_raises():
    # drone 2's goal sits 0.1 m from drone 1's parked position
    with pytest.raises(PlanningError, match="drone 2"):
        plan_paths([(0.0, 0.0), (-4.0, 0.0)],
                   [(0.0, 0.0), (0.1, 0.0)], ARENA)


def test_start_outside_arena_is_clamped():
    paths, _ = plan_paths([(-6.0, 0.0)], [(4.0, 0.0)], ARENA)
    assert paths[0][0] == (-5.0, 0.0)


def test_eight_drones_line_swap():
    ys = [-3.5 + i for i in range(8)]
    starts = [(-4.0, y) for y in ys]
    goals = [(4.0, y) for y in reversed(ys)]
    paths, _ = plan_paths(starts, goals, ARENA)
    assert len(paths) == 8
    for k, path in enumerate(paths):
        assert path[0] == starts[k]
        assert path[-1] == goals[k]
        for (x, y) in path:
            assert ARENA.contains(x, y)


def test_one_drone_same_start_and_goal():
    paths, _ = plan_paths([(1.0, 1.0)], [(1.0, 1.0)], ARENA)
    assert paths[0][0] == (1.0, 1.0)
    assert paths[0][-1] == (1.0, 1.0)
