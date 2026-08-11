"""Pure rendering of one drone's remote flight parameters.

No ROS dependencies, no side effects other than :func:`write_params_yaml`'s
explicit file write - unit-tested standalone like ``formations.py``.

Per the mission requirements ("generate a launch file... copy to drone...
execute"), the concrete artifact generated and copied is a rendered params
YAML in the ``drones.yaml``-style ``/**: ros__parameters:`` wildcard shape
(this file IS loaded via ``declare_parameter``, by ``flight_executor_node``
on the companion computer - unlike ``flight_profiles.yaml``/
``drone_profiles.yaml``/``mission.yaml``, which are read directly by plain
Python and never wrapped this way). ``single_drone_flight.launch.py`` is a
static, checked-in launch file that accepts this file as a ``params_file``
argument, rather than a per-mission regenerated ``.launch.py``.
"""

from pathlib import Path
from typing import Dict

import yaml

from formation_interface.drone_profiles import DroneProfile
from formation_interface.flight_profiles import FlightProfile


def render_single_drone_params(drone_profile: DroneProfile, flight_profile: FlightProfile,
                                mission_name: str, run_id: str) -> Dict:
    """Build the flat parameter dict ``flight_executor_node`` declares/reads."""
    return {
        "mission_name": mission_name,
        "run_id": run_id,
        "drone_name": drone_profile.name,
        "namespace": drone_profile.namespace,
        "system_id": drone_profile.system_id,
        "pose_topic": drone_profile.pose_topic,
        "pose_topic_type": drone_profile.pose_topic_type,
        "state": flight_profile.state,
        "control_scheme": flight_profile.control_scheme,
        "flight_profile_name": flight_profile.name,
        "params": dict(flight_profile.params),
        "trajectory": dict(flight_profile.trajectory),
        "max_speed": drone_profile.max_speed,
        "max_accel": drone_profile.max_accel,
        "remote_log_dir": drone_profile.remote_log_dir,
    }


def write_params_yaml(params: Dict, dest_path: Path) -> Path:
    """Write ``params`` wrapped in the ``/**: ros__parameters:`` wildcard
    shape to ``dest_path`` (parent directories created as needed)."""
    dest_path = Path(dest_path)
    dest_path.parent.mkdir(parents=True, exist_ok=True)
    doc = {"/**": {"ros__parameters": params}}
    dest_path.write_text(yaml.safe_dump(doc, sort_keys=False))
    return dest_path
