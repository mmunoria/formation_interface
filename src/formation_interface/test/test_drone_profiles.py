import pytest

from formation_interface.drone_profiles import (
    drone_supports,
    list_drone_profiles,
    load_drone_profile,
)


def _write(tmp_path, name, content):
    path = tmp_path / name
    path.write_text(content)
    return path


def test_load_drone_profile_happy_path(tmp_path):
    path = _write(tmp_path, "drone1.yaml", """
name: drone1
namespace: px4_1
system_id: 2
pose_topic: "/vrpn_mocap/drone1/pose"
capabilities: ["gps", "camera"]
""")
    profile = load_drone_profile(path)
    assert profile.name == "drone1"
    assert profile.namespace == "px4_1"
    assert profile.system_id == 2
    assert profile.capabilities == ["gps", "camera"]
    assert profile.backend == "mock_ssh"   # default


def test_load_drone_profile_missing_field_raises(tmp_path):
    path = _write(tmp_path, "bad.yaml", "name: bad\nnamespace: px4_1\n")
    with pytest.raises(ValueError, match="missing required field"):
        load_drone_profile(path)


def test_list_drone_profiles_duplicate_name_raises(tmp_path):
    _write(tmp_path, "a.yaml",
           'name: dup\nnamespace: px4_1\nsystem_id: 2\npose_topic: "/p"\n')
    _write(tmp_path, "b.yaml",
           'name: dup\nnamespace: px4_2\nsystem_id: 3\npose_topic: "/p2"\n')
    with pytest.raises(ValueError, match="duplicate drone profile name"):
        list_drone_profiles(tmp_path)


def test_list_drone_profiles_missing_dir_returns_empty(tmp_path):
    assert list_drone_profiles(tmp_path / "nope") == {}


def test_drone_supports_subset_logic(tmp_path):
    path = _write(
        tmp_path, "d.yaml",
        'name: d\nnamespace: px4_1\nsystem_id: 2\npose_topic: "/p"\n'
        'capabilities: ["gps", "camera"]\n')
    profile = load_drone_profile(path)
    assert drone_supports(profile, [])
    assert drone_supports(profile, ["gps"])
    assert drone_supports(profile, ["gps", "camera"])
    assert not drone_supports(profile, ["lidar"])
