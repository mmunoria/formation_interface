import math

from formation_interface.active_tracker import DroneRegistry, quat_to_yaw


def test_unknown_drone_is_none():
    reg = DroneRegistry()
    assert reg.get(7) is None
    assert reg.active_drones() == []


def test_pose_then_active_creates_one_drone():
    reg = DroneRegistry()
    reg.update_pose(3, 1.0, 2.0)
    reg.update_active(3, True, now=0.0)
    drones = reg.active_drones()
    assert len(drones) == 1
    assert drones[0].id == 3
    assert drones[0].position == (1.0, 2.0)


def test_active_false_removes_from_active_list_but_keeps_state():
    reg = DroneRegistry()
    reg.update_pose(1, 5.0, 5.0)
    reg.update_active(1, True, now=0.0)
    reg.update_active(1, False, now=1.0)
    assert reg.active_drones() == []
    assert reg.get(1).position == (5.0, 5.0)


def test_prune_stale_deactivates_silent_drone():
    reg = DroneRegistry(stale_timeout=2.0)
    reg.update_active(9, True, now=0.0)
    reg.prune_stale(now=1.0)
    assert reg.get(9).active is True
    reg.prune_stale(now=5.0)
    assert reg.get(9).active is False


def test_prune_stale_ignores_drones_still_within_timeout():
    reg = DroneRegistry(stale_timeout=2.0)
    reg.update_active(4, True, now=0.0)
    reg.update_active(4, True, now=1.5)   # heartbeat refreshes last_seen
    reg.prune_stale(now=3.0)
    assert reg.get(4).active is True


def test_set_goal_and_set_path():
    reg = DroneRegistry()
    reg.update_active(2, True, now=0.0)
    reg.set_goal(2, (1.0, 1.0))
    reg.set_path(2, [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)])
    drone = reg.get(2)
    assert drone.goal == (1.0, 1.0)
    assert drone.planned_path == [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)]


def test_set_path_none_clears_it():
    reg = DroneRegistry()
    reg.update_active(2, True, now=0.0)
    reg.set_path(2, [(0.0, 0.0), (1.0, 1.0)])
    reg.set_path(2, None)
    assert reg.get(2).planned_path == []


def test_quat_to_yaw_identity_is_zero():
    assert abs(quat_to_yaw(0.0, 0.0, 0.0, 1.0)) < 1e-9


def test_quat_to_yaw_ninety_degrees():
    # 90 deg rotation about +Z.
    half = math.pi / 4.0
    yaw = quat_to_yaw(0.0, 0.0, math.sin(half), math.cos(half))
    assert abs(yaw - math.pi / 2.0) < 1e-9


def test_drone_yaw_property_matches_orientation():
    reg = DroneRegistry()
    half = math.pi / 4.0
    reg.update_pose(1, 0.0, 0.0, orientation=(0.0, 0.0, math.sin(half), math.cos(half)))
    assert abs(reg.get(1).yaw - math.pi / 2.0) < 1e-9
