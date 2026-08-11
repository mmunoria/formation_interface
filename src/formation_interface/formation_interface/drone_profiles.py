"""Drone profiles: drone-specific parameters that are NOT mission-specific.

No ROS dependencies live here on purpose (same pattern as ``flight_profiles.py``).
One file per physical drone under ``config/drone_profiles/`` - unlike
``config/drones.yaml``'s index-aligned parallel lists, so adding, editing, or
retiring one drone touches exactly one small file (the "quickly varying
assignment" requirement) and carries its own ``capabilities`` bag for
matching against a flight profile's ``required_capabilities``.

``config/drones.yaml`` is untouched by this module - it keeps serving
``formation_node``/``gui_node`` exactly as before; drone profiles are
read only by the mission-manager subsystem.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Sequence

import yaml


@dataclass
class DroneProfile:
    name: str
    namespace: str
    system_id: int
    pose_topic: str
    pose_topic_type: str = "PoseStamped"
    capabilities: List[str] = field(default_factory=list)
    max_speed: float = 2.0
    max_accel: float = 1.5
    backend: str = "mock_ssh"
    ssh_host: str = ""
    ssh_user: str = ""
    ssh_key_path: str = ""
    remote_workspace_path: str = ""
    remote_log_dir: str = ""


_REQUIRED_FIELDS = ("name", "namespace", "system_id", "pose_topic")


def load_drone_profile(path: Path) -> DroneProfile:
    """Load and validate one drone profile YAML file.

    Raises:
        ValueError: a required field is missing.
    """
    path = Path(path)
    data = yaml.safe_load(path.read_text()) or {}

    missing = [f for f in _REQUIRED_FIELDS if f not in data]
    if missing:
        raise ValueError(f"{path}: missing required field(s) {missing}")

    return DroneProfile(
        name=data["name"],
        namespace=data["namespace"],
        system_id=int(data["system_id"]),
        pose_topic=data["pose_topic"],
        pose_topic_type=data.get("pose_topic_type", "PoseStamped"),
        capabilities=list(data.get("capabilities", [])),
        max_speed=float(data.get("max_speed", 2.0)),
        max_accel=float(data.get("max_accel", 1.5)),
        backend=data.get("backend", "mock_ssh"),
        ssh_host=data.get("ssh_host", ""),
        ssh_user=data.get("ssh_user", ""),
        ssh_key_path=data.get("ssh_key_path", ""),
        remote_workspace_path=data.get("remote_workspace_path", ""),
        remote_log_dir=data.get("remote_log_dir", ""),
    )


def list_drone_profiles(dir_: Path) -> Dict[str, DroneProfile]:
    """Load every ``*.yaml`` file in ``dir_``, keyed by each profile's `name`.

    Raises:
        ValueError: two files declare the same `name`.
    """
    dir_ = Path(dir_)
    profiles: Dict[str, DroneProfile] = {}
    if not dir_.is_dir():
        return profiles
    for path in sorted(dir_.glob("*.yaml")):
        profile = load_drone_profile(path)
        if profile.name in profiles:
            raise ValueError(
                f"duplicate drone profile name '{profile.name}' "
                f"({path} and a previously loaded file)")
        profiles[profile.name] = profile
    return profiles


def drone_supports(profile: DroneProfile, required: Sequence[str]) -> bool:
    """True if ``profile`` has every capability in ``required``."""
    return set(required).issubset(set(profile.capabilities))
