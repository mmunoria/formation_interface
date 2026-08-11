import yaml

from formation_interface.deploy import launch_gen
from formation_interface.drone_profiles import DroneProfile
from formation_interface.flight_profiles import FlightProfile


def _drone():
    return DroneProfile(
        name="drone1", namespace="px4_1", system_id=2,
        pose_topic="/vrpn_mocap/drone1/pose", capabilities=["gps"],
        remote_log_dir="/home/px4/logs")


def _flight():
    return FlightProfile(
        name="hover_low", description="hold", state="OFFBOARD",
        control_scheme="position", params={"altitude": 1.5},
        trajectory={"type": "static_point", "offset": [0.0, 0.0, 1.5]})


def test_render_single_drone_params_shape():
    params = launch_gen.render_single_drone_params(
        _drone(), _flight(), "test_mission", "run1")
    assert params["drone_name"] == "drone1"
    assert params["namespace"] == "px4_1"
    assert params["system_id"] == 2
    assert params["state"] == "OFFBOARD"
    assert params["control_scheme"] == "position"
    assert params["flight_profile_name"] == "hover_low"
    assert params["trajectory"]["type"] == "static_point"
    assert params["mission_name"] == "test_mission"
    assert params["run_id"] == "run1"


def test_write_params_yaml_round_trips_wildcard_shape(tmp_path):
    params = launch_gen.render_single_drone_params(_drone(), _flight(), "m", "r1")
    dest = tmp_path / "params" / "drone1_params.yaml"
    written = launch_gen.write_params_yaml(params, dest)
    assert written == dest
    assert dest.exists()

    loaded = yaml.safe_load(dest.read_text())
    assert "/**" in loaded
    assert loaded["/**"]["ros__parameters"]["drone_name"] == "drone1"
