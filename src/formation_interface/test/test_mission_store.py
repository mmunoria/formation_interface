import pytest

from formation_interface.mission_store import (
    MissionExistsError,
    create_mission,
    list_missions,
    load_mission,
    new_run_dir,
    save_mission,
)


def test_create_mission_happy_path(tmp_path):
    mission_dir = create_mission(
        tmp_path, "test_mission", description="a test",
        assignment={"drone1": "hover_low"})
    assert mission_dir == tmp_path / "test_mission"
    assert (mission_dir / "mission.yaml").exists()
    assert (mission_dir / "README.md").exists()

    spec = load_mission(tmp_path, "test_mission")
    assert spec.name == "test_mission"
    assert spec.description == "a test"
    assert spec.assignment == {"drone1": "hover_low"}
    assert spec.features["auto_arm"] is False   # default feature present


def test_create_mission_twice_raises_without_force(tmp_path):
    create_mission(tmp_path, "dup")
    with pytest.raises(MissionExistsError):
        create_mission(tmp_path, "dup")


def test_create_mission_force_overwrites_yaml_but_not_readme(tmp_path):
    create_mission(tmp_path, "m", description="first")
    readme_before = (tmp_path / "m" / "README.md").read_text()

    create_mission(tmp_path, "m", description="second", force=True)
    spec = load_mission(tmp_path, "m")
    assert spec.description == "second"
    assert (tmp_path / "m" / "README.md").read_text() == readme_before


def test_save_mission_never_touches_readme(tmp_path):
    create_mission(tmp_path, "m")
    readme_before = (tmp_path / "m" / "README.md").read_text()

    spec = load_mission(tmp_path, "m")
    spec.assignment = {"drone1": "hover_low"}
    save_mission(tmp_path, spec)

    assert load_mission(tmp_path, "m").assignment == {"drone1": "hover_low"}
    assert (tmp_path / "m" / "README.md").read_text() == readme_before


def test_list_missions_only_lists_dirs_with_mission_yaml(tmp_path):
    create_mission(tmp_path, "a")
    create_mission(tmp_path, "b")
    (tmp_path / "not_a_mission").mkdir()
    assert list_missions(tmp_path) == ["a", "b"]


def test_list_missions_missing_root_returns_empty(tmp_path):
    assert list_missions(tmp_path / "nope") == []


def test_new_run_dir_never_collides(tmp_path):
    mission_dir = create_mission(tmp_path, "m")
    run1 = new_run_dir(mission_dir)
    run2 = new_run_dir(mission_dir)
    assert run1 != run2
    assert run1.exists() and run2.exists()
    assert run1.parent == mission_dir / "runs"


def test_load_mission_missing_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_mission(tmp_path, "does_not_exist")
