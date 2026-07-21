"""Live drone bookkeeping - no ROS dependencies on purpose.

Tracks whatever drones happen to be reporting in right now (identified by an
ID carried in the message, not by position in a config list), so it can be
unit tested on its own and reused by any front-end (the monitor GUI today,
something else later).

A :class:`Drone` is created the first time either its pose or its active
status is seen, and is kept (last known state) even once inactive - only
:meth:`DroneRegistry.active_drones` filters it out, so a reconnect doesn't
lose its last position/path.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Sequence, Tuple

Point2D = Tuple[float, float]
Quaternion = Tuple[float, float, float, float]


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract the Z-axis (yaw) rotation from a quaternion, in radians."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class Drone:
    id: int
    position: Point2D = (0.0, 0.0)
    orientation: Quaternion = (0.0, 0.0, 0.0, 1.0)
    active: bool = False
    planned_path: List[Point2D] = field(default_factory=list)
    goal: Optional[Point2D] = None

    @property
    def yaw(self) -> float:
        return quat_to_yaw(*self.orientation)


class DroneRegistry:
    """All known drones, keyed by ID.

    ``stale_timeout`` is a safety fallback: if a drone stops sending
    ``DroneActive`` heartbeats (e.g. it crashed) for that many seconds, it is
    force-deactivated even without an explicit ``active: false``.
    """

    def __init__(self, stale_timeout: float = 2.0):
        self.stale_timeout = float(stale_timeout)
        self._drones: Dict[int, Drone] = {}
        self._last_active_seen: Dict[int, float] = {}

    def _get_or_create(self, drone_id: int) -> Drone:
        drone = self._drones.get(drone_id)
        if drone is None:
            drone = Drone(id=drone_id)
            self._drones[drone_id] = drone
        return drone

    # ------------------------------ intake --------------------------------
    def update_pose(
        self,
        drone_id: int,
        x: float,
        y: float,
        orientation: Quaternion = (0.0, 0.0, 0.0, 1.0),
    ) -> None:
        drone = self._get_or_create(drone_id)
        drone.position = (float(x), float(y))
        drone.orientation = tuple(float(v) for v in orientation)

    def update_active(self, drone_id: int, active: bool, now: float) -> None:
        drone = self._get_or_create(drone_id)
        drone.active = bool(active)
        if drone.active:
            self._last_active_seen[drone_id] = float(now)

    def prune_stale(self, now: float) -> None:
        for drone_id, last_seen in self._last_active_seen.items():
            if now - last_seen > self.stale_timeout:
                self._drones[drone_id].active = False

    # ------------------------------ queries --------------------------------
    def get(self, drone_id: int) -> Optional[Drone]:
        return self._drones.get(drone_id)

    def all_drones(self) -> List[Drone]:
        return list(self._drones.values())

    def active_drones(self) -> List[Drone]:
        return [d for d in self._drones.values() if d.active]

    # ------------------------------ planning state --------------------------
    def set_goal(self, drone_id: int, goal: Optional[Point2D]) -> None:
        drone = self._drones.get(drone_id)
        if drone is not None:
            drone.goal = None if goal is None else (float(goal[0]), float(goal[1]))

    def set_path(self, drone_id: int, path: Optional[Sequence[Point2D]]) -> None:
        drone = self._drones.get(drone_id)
        if drone is not None:
            drone.planned_path = (
                [] if not path else [(float(p[0]), float(p[1])) for p in path]
            )
