"""Pure-geometry formation generators.

No ROS dependencies live here on purpose: this module is imported by the
formation node *and* the interface node (for the menu), and it can be unit
tested on its own.

Three built-in formations (``line``, ``v``, ``circle``) each return an (n, 2)
array of XY offsets in a *formation-local* frame whose origin is the formation
centre and whose +X axis points along the formation heading.

Arbitrary user-defined formations are supported through
:func:`transform_offsets`, which places *any* list of per-drone offsets
(built-in or entered through the interface) into the world frame: rotate by
``yaw``, translate to ``center``, stamp ``altitude`` (plus an optional per-drone
z offset, so custom formations may be 3-D).
"""

import math
from typing import List, Sequence, Tuple

import numpy as np

Position = Tuple[float, float, float]


def _line(n: int, spacing: float) -> np.ndarray:
    """Side-by-side, spread along local +/-Y (a horizontal row)."""
    ys = (np.arange(n) - (n - 1) / 2.0) * spacing
    return np.column_stack((np.zeros(n), ys))


def _v(n: int, spacing: float) -> np.ndarray:
    """Flying-V: leader at the front tip, drones trail back on two arms."""
    offs = [(0.0, 0.0)]
    left = right = 0
    for i in range(1, n):
        if i % 2 == 1:
            left += 1
            offs.append((-left * spacing, left * spacing))      # back-left arm
        else:
            right += 1
            offs.append((-right * spacing, -right * spacing))   # back-right arm
    return np.array(offs, dtype=float)


def _circle(n: int, spacing: float) -> np.ndarray:
    """Evenly spaced on a circle; ``spacing`` is the chord between neighbours."""
    if n == 1:
        return np.zeros((1, 2))
    radius = spacing / (2.0 * math.sin(math.pi / n))
    ang = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return np.column_stack((radius * np.cos(ang), radius * np.sin(ang)))


_GENERATORS = {
    "line": _line,
    "v": _v,
    "circle": _circle,
}

# Public list of built-in formation names (used by the interface menu).
FORMATIONS: Tuple[str, ...] = tuple(sorted(_GENERATORS))

# Reserved name for user-defined formations sent with explicit offsets.
CUSTOM = "custom"


def transform_offsets(
    offsets: Sequence[Sequence[float]],
    center: Sequence[float],
    altitude: float,
    yaw: float = 0.0,
) -> List[Position]:
    """Place formation-local offsets into the world/ENU frame.

    Args:
        offsets:  per-drone (x, y) or (x, y, dz) offsets from the formation
                  centre, metres; ``dz`` is added on top of ``altitude``.
        center:   (x, y[, z]) centre of the formation in the world frame.
        altitude: base altitude (z), positive up.
        yaw:      heading the formation faces, radians (CCW from +X/east).
    """
    c, s = math.cos(yaw), math.sin(yaw)
    out: List[Position] = []
    for off in offsets:
        x, y = float(off[0]), float(off[1])
        dz = float(off[2]) if len(off) > 2 else 0.0
        wx = c * x - s * y + float(center[0])
        wy = s * x + c * y + float(center[1])
        out.append((wx, wy, float(altitude) + dz))
    return out


def compute_targets(
    formation: str,
    n: int,
    spacing: float,
    center: Sequence[float],
    altitude: float,
    yaw: float = 0.0,
) -> List[Position]:
    """Return a list of ``n`` (x, y, z) world/ENU targets for a built-in formation.

    Raises:
        ValueError: if ``formation`` is not one of :data:`FORMATIONS`.
    """
    key = formation.lower().strip()
    if key not in _GENERATORS:
        raise ValueError(
            f"unknown formation '{formation}'; choose from {list(FORMATIONS)}"
        )
    if n <= 0:
        return []

    local = _GENERATORS[key](n, float(spacing))        # (n, 2) local XY
    return transform_offsets(local.tolist(), center, altitude, yaw)
