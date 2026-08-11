"""Mission definitions: naming, assignment, anti-overwrite, README, run logs.

No ROS dependencies live here on purpose (same pattern as ``flight_profiles.py``
/ ``drone_profiles.py``). A mission pairs a name/description with an
assignment of drone_name -> flight_profile name, plus feature toggles. It is
written once under ``missions/<name>/`` and never silently overwritten: a
plain directory-existence check (see :func:`create_mission`) is the entire
anti-overwrite mechanic. Each Start gets its own timestamped run directory
under ``runs/`` that is never reused, so re-running a mission never
overwrites a previous run's params/logs either.
"""

from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List

import yaml

DEFAULT_FEATURES = {
    "auto_arm": False,
    "pull_logs_on_terminate": True,
    "record_bag": False,
}


class MissionExistsError(Exception):
    """Raised by :func:`create_mission` when the mission directory already
    exists and ``force`` was not given - the anti-overwrite guard."""


@dataclass
class MissionSpec:
    name: str
    description: str = ""
    created_at: str = ""
    author: str = ""
    assignment: Dict[str, str] = field(default_factory=dict)   # drone_name -> flight_profile name
    features: Dict[str, bool] = field(default_factory=lambda: dict(DEFAULT_FEATURES))


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def create_mission(root: Path, name: str, description: str = "", author: str = "",
                    assignment: Dict[str, str] = None,
                    features: Dict[str, bool] = None,
                    force: bool = False) -> Path:
    """Create ``root/name/`` with a fresh ``mission.yaml`` and ``README.md``.

    Raises:
        MissionExistsError: ``root/name/`` already exists and ``force`` is False.
    """
    root = Path(root)
    mission_dir = root / name
    if mission_dir.exists() and not force:
        raise MissionExistsError(
            f"mission '{name}' already exists at {mission_dir} "
            f"(pass force=True to overwrite its mission.yaml)")

    mission_dir.mkdir(parents=True, exist_ok=True)
    spec = MissionSpec(
        name=name,
        description=description,
        created_at=_utc_now_iso(),
        author=author,
        assignment=dict(assignment or {}),
        features={**DEFAULT_FEATURES, **(features or {})},
    )
    save_mission(root, spec)
    write_readme_if_absent(mission_dir, spec)
    return mission_dir


def load_mission(root: Path, name: str) -> MissionSpec:
    path = Path(root) / name / "mission.yaml"
    if not path.exists():
        raise FileNotFoundError(f"no mission named '{name}' under {root}")
    data = yaml.safe_load(path.read_text()) or {}
    return MissionSpec(
        name=data.get("name", name),
        description=data.get("description", ""),
        created_at=data.get("created_at", ""),
        author=data.get("author", ""),
        assignment=dict(data.get("assignment", {})),
        features={**DEFAULT_FEATURES, **data.get("features", {})},
    )


def save_mission(root: Path, spec: MissionSpec) -> None:
    """Overwrite ``mission.yaml`` only - never touches ``README.md``."""
    mission_dir = Path(root) / spec.name
    mission_dir.mkdir(parents=True, exist_ok=True)
    data = {
        "name": spec.name,
        "description": spec.description,
        "created_at": spec.created_at,
        "author": spec.author,
        "assignment": spec.assignment,
        "features": spec.features,
    }
    (mission_dir / "mission.yaml").write_text(yaml.safe_dump(data, sort_keys=False))


def list_missions(root: Path) -> List[str]:
    root = Path(root)
    if not root.is_dir():
        return []
    return sorted(
        p.name for p in root.iterdir()
        if p.is_dir() and (p / "mission.yaml").exists())


def write_readme_if_absent(mission_dir: Path, spec: MissionSpec) -> None:
    readme = Path(mission_dir) / "README.md"
    if readme.exists():
        return
    readme.write_text(
        f"# {spec.name}\n\n"
        f"{spec.description}\n\n"
        f"Created: {spec.created_at} by {spec.author or 'unknown'}\n")


def new_run_dir(mission_dir: Path) -> Path:
    """A fresh ``runs/<UTC-timestamp>/`` directory, never reused: on a
    same-second collision, retries with a ``-2``, ``-3``, ... suffix."""
    runs = Path(mission_dir) / "runs"
    runs.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")

    candidate = runs / stamp
    suffix = 1
    while True:
        try:
            candidate.mkdir(parents=False, exist_ok=False)
            return candidate
        except FileExistsError:
            suffix += 1
            candidate = runs / f"{stamp}-{suffix}"
