"""Flight profiles: non-drone-specific description of how to fly.

No ROS dependencies live here on purpose (same pattern as ``formations.py``
and ``planning.py``): this module is imported by ``mission_node`` *and*
``mission_gui``, and it is unit-tested standalone.

A flight profile names a target PX4 flight *state* (nav_state), a *control
scheme* (which offboard setpoint type to stream), a bag of non-drone-specific
*params*, and a time-parametric *reference trajectory* sampled relative to
whatever "base" position the caller supplies (typically a drone's home
position). It says nothing about which drone flies it or what that drone's
namespace/system_id/capabilities are - that's ``drone_profiles.py``.

Note these per-drone, time-over-flight generators are a different thing from
``formations.py``'s per-swarm, single-instant spatial-offset generators
(``_line``/``_v``/``_circle``): one is "where is drone i within the swarm
right now", the other is "where should this one drone be at time t".
"""

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import yaml

Position = Tuple[float, float, float, float]   # x, y, z, yaw (ENU)

# Mirrors px4_msgs/msg/VehicleStatus.msg's NAVIGATION_STATE_* constants.
# Hand-kept in sync with src/px4_msgs/msg/VehicleStatus.msg - if PX4's enum
# changes, update both this dict and formation_interfaces/msg/DroneTelemetry.msg.
NAV_STATE_NAMES = {
    "MANUAL": 0,
    "ALTCTL": 1,
    "POSCTL": 2,
    "AUTO_MISSION": 3,
    "AUTO_LOITER": 4,
    "AUTO_RTL": 5,
    "ACRO": 10,
    "DESCEND": 12,
    "TERMINATION": 13,
    "OFFBOARD": 14,
    "STAB": 15,
    "AUTO_TAKEOFF": 17,
    "AUTO_LAND": 18,
}

CONTROL_SCHEMES = ("position", "velocity", "trajectory")

_DEFAULT_TRAJECTORY = {"type": "static_point", "offset": [0.0, 0.0, 0.0]}


@dataclass
class FlightProfile:
    name: str
    description: str
    state: str
    control_scheme: str
    params: dict = field(default_factory=dict)
    trajectory: dict = field(default_factory=lambda: dict(_DEFAULT_TRAJECTORY))
    required_capabilities: List[str] = field(default_factory=list)


def load_flight_profile(path: Path) -> FlightProfile:
    """Load and validate one flight profile YAML file.

    Raises:
        ValueError: unknown ``state``/``control_scheme``, or a missing
            required field.
    """
    path = Path(path)
    data = yaml.safe_load(path.read_text()) or {}

    name = data.get("name") or path.stem
    state = data.get("state", "OFFBOARD")
    if state not in NAV_STATE_NAMES:
        raise ValueError(
            f"{path}: unknown state '{state}'; choose from {sorted(NAV_STATE_NAMES)}")

    control_scheme = data.get("control_scheme", "position")
    if control_scheme not in CONTROL_SCHEMES:
        raise ValueError(
            f"{path}: unknown control_scheme '{control_scheme}'; "
            f"choose from {CONTROL_SCHEMES}")

    return FlightProfile(
        name=name,
        description=data.get("description", ""),
        state=state,
        control_scheme=control_scheme,
        params=dict(data.get("params", {})),
        trajectory=dict(data.get("trajectory", _DEFAULT_TRAJECTORY)),
        required_capabilities=list(data.get("required_capabilities", [])),
    )


def list_flight_profiles(dir_: Path) -> Dict[str, FlightProfile]:
    """Load every ``*.yaml`` file in ``dir_``, keyed by each profile's `name`.

    Raises:
        ValueError: two files declare the same `name`.
    """
    dir_ = Path(dir_)
    profiles: Dict[str, FlightProfile] = {}
    if not dir_.is_dir():
        return profiles
    for path in sorted(dir_.glob("*.yaml")):
        profile = load_flight_profile(path)
        if profile.name in profiles:
            raise ValueError(
                f"duplicate flight profile name '{profile.name}' "
                f"({path} and a previously loaded file)")
        profiles[profile.name] = profile
    return profiles


def sample_trajectory(spec: dict, t: float, base: Sequence[float]) -> Position:
    """Sample a reference trajectory at time ``t`` (seconds since flight
    start), relative to ``base`` = (x, y, z) world/ENU. Returns (x, y, z, yaw).

    Raises:
        ValueError: unknown ``spec['type']`` or a malformed spec.
    """
    kind = spec.get("type", "static_point")
    if kind == "static_point":
        return _sample_static_point(spec, base)
    if kind == "circle":
        return _sample_circle(spec, t, base)
    if kind == "waypoints":
        return _sample_waypoints(spec, t, base)
    raise ValueError(f"unknown trajectory type '{kind}'")


def _sample_static_point(spec: dict, base: Sequence[float]) -> Position:
    dx, dy, dz = spec.get("offset", [0.0, 0.0, 0.0])
    yaw = float(spec.get("yaw", 0.0))
    return (base[0] + dx, base[1] + dy, base[2] + dz, yaw)


def _sample_circle(spec: dict, t: float, base: Sequence[float]) -> Position:
    radius = float(spec.get("radius", 1.0))
    period = float(spec.get("period", 20.0))
    if period <= 0:
        raise ValueError("circle trajectory 'period' must be > 0")
    dx, dy, dz = spec.get("offset", [0.0, 0.0, 0.0])
    ang = 2.0 * math.pi * (t / period)
    x = base[0] + dx + radius * math.cos(ang)
    y = base[1] + dy + radius * math.sin(ang)
    z = base[2] + dz
    yaw = ang + math.pi / 2.0    # face tangent to the circle
    return (x, y, z, yaw)


def _sample_waypoints(spec: dict, t: float, base: Sequence[float]) -> Position:
    points = spec.get("points")
    if not points:
        raise ValueError("waypoints trajectory needs a non-empty 'points' list")
    pts = sorted(points, key=lambda p: p[3])   # each: [x, y, z, t_seconds]

    if t <= pts[0][3]:
        x, y, z, _ = pts[0]
        return (base[0] + x, base[1] + y, base[2] + z, 0.0)
    if t >= pts[-1][3]:
        x, y, z, _ = pts[-1]
        return (base[0] + x, base[1] + y, base[2] + z, 0.0)

    for (x0, y0, z0, t0), (x1, y1, z1, t1) in zip(pts, pts[1:]):
        if t0 <= t <= t1:
            frac = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            x = x0 + frac * (x1 - x0)
            y = y0 + frac * (y1 - y0)
            z = z0 + frac * (z1 - z0)
            return (base[0] + x, base[1] + y, base[2] + z, 0.0)

    raise ValueError("waypoint interpolation failed")   # unreachable
