import pytest

from formation_interface.flight_profiles import (
    NAV_STATE_NAMES,
    list_flight_profiles,
    load_flight_profile,
    sample_trajectory,
)


def _write(tmp_path, name, content):
    path = tmp_path / name
    path.write_text(content)
    return path


def test_nav_state_names_matches_px4_offboard_value():
    assert NAV_STATE_NAMES["OFFBOARD"] == 14


def test_load_flight_profile_happy_path(tmp_path):
    path = _write(tmp_path, "hover.yaml", """
name: hover
description: hold position
state: OFFBOARD
control_scheme: position
params:
  altitude: 1.5
trajectory:
  type: static_point
  offset: [0.0, 0.0, 1.5]
required_capabilities: []
""")
    profile = load_flight_profile(path)
    assert profile.name == "hover"
    assert profile.state == "OFFBOARD"
    assert profile.control_scheme == "position"
    assert profile.params["altitude"] == 1.5
    assert profile.trajectory["type"] == "static_point"


def test_unknown_state_raises(tmp_path):
    path = _write(tmp_path, "bad.yaml", "name: bad\nstate: NOT_A_STATE\n")
    with pytest.raises(ValueError, match="unknown state"):
        load_flight_profile(path)


def test_unknown_control_scheme_raises(tmp_path):
    path = _write(
        tmp_path, "bad.yaml",
        "name: bad\nstate: OFFBOARD\ncontrol_scheme: warp\n")
    with pytest.raises(ValueError, match="unknown control_scheme"):
        load_flight_profile(path)


def test_list_flight_profiles_keyed_by_name_not_filename(tmp_path):
    _write(tmp_path, "a.yaml", "name: real_name\nstate: OFFBOARD\n")
    profiles = list_flight_profiles(tmp_path)
    assert set(profiles) == {"real_name"}


def test_list_flight_profiles_duplicate_name_raises(tmp_path):
    _write(tmp_path, "a.yaml", "name: dup\nstate: OFFBOARD\n")
    _write(tmp_path, "b.yaml", "name: dup\nstate: OFFBOARD\n")
    with pytest.raises(ValueError, match="duplicate flight profile name"):
        list_flight_profiles(tmp_path)


def test_list_flight_profiles_missing_dir_returns_empty(tmp_path):
    assert list_flight_profiles(tmp_path / "does_not_exist") == {}


def test_sample_static_point():
    x, y, z, yaw = sample_trajectory(
        {"type": "static_point", "offset": [1.0, 0.0, 0.5]}, 5.0, (0.0, 0.0, 1.0))
    assert (x, y, z) == (1.0, 0.0, 1.5)
    assert yaw == 0.0


def test_sample_circle_returns_to_start_after_one_period():
    spec = {"type": "circle", "radius": 2.0, "period": 10.0, "offset": [0.0, 0.0, 0.0]}
    x0, y0, z0, _ = sample_trajectory(spec, 0.0, (0.0, 0.0, 0.0))
    x1, y1, z1, _ = sample_trajectory(spec, 10.0, (0.0, 0.0, 0.0))
    assert abs(x0 - x1) < 1e-9
    assert abs(y0 - y1) < 1e-9
    assert z0 == z1


def test_sample_circle_zero_period_raises():
    with pytest.raises(ValueError, match="period"):
        sample_trajectory({"type": "circle", "period": 0.0}, 0.0, (0.0, 0.0, 0.0))


def test_sample_waypoints_interpolates_linearly():
    spec = {"type": "waypoints",
            "points": [[0.0, 0.0, 0.0, 0.0], [10.0, 0.0, 0.0, 10.0]]}
    x, y, z, _ = sample_trajectory(spec, 5.0, (0.0, 0.0, 0.0))
    assert abs(x - 5.0) < 1e-9


def test_sample_waypoints_clamps_before_and_after():
    spec = {"type": "waypoints",
            "points": [[0.0, 0.0, 0.0, 5.0], [10.0, 0.0, 0.0, 15.0]]}
    x_before, *_ = sample_trajectory(spec, 0.0, (0.0, 0.0, 0.0))
    x_after, *_ = sample_trajectory(spec, 100.0, (0.0, 0.0, 0.0))
    assert x_before == 0.0
    assert x_after == 10.0


def test_sample_waypoints_empty_raises():
    with pytest.raises(ValueError, match="non-empty"):
        sample_trajectory({"type": "waypoints", "points": []}, 0.0, (0.0, 0.0, 0.0))


def test_unknown_trajectory_type_raises():
    with pytest.raises(ValueError, match="unknown trajectory type"):
        sample_trajectory({"type": "warp_drive"}, 0.0, (0.0, 0.0, 0.0))
