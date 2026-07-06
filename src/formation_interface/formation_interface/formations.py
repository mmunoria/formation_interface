"""Pure-geometry formation generators.

No ROS dependencies live here on purpose: this module is imported by the
formation node *and* the interface node (for the menu), and it can be unit
tested on its own.

Each generator returns an (n, 2) array of XY offsets in a *formation-local*
frame whose origin is the formation centre and whose +X axis points along the
formation heading.  ``compute_targets`` then rotates by ``yaw``, translates to
``center`` and stamps every drone with the requested ``altitude``.
"""

import math
from typing import List, Sequence, Tuple

import numpy as np

Position = Tuple[float, float, float]


def _line(n: int, spacing: float) -> np.ndarray:
    """Side-by-side, spread along local +/-Y (a horizontal row)."""
    ys = (np.arange(n) - (n - 1) / 2.0) * spacing
    return np.column_stack((np.zeros(n), ys))


def _column(n: int, spacing: float) -> np.ndarray:
    """Single file, spread along local +/-X (front-to-back)."""
    xs = -(np.arange(n) - (n - 1) / 2.0) * spacing
    return np.column_stack((xs, np.zeros(n)))


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


def _grid(n: int, spacing: float) -> np.ndarray:
    """Near-square grid, centred on the origin."""
    cols = int(math.ceil(math.sqrt(n)))
    offs = np.array(
        [((i // cols) * spacing, (i % cols) * spacing) for i in range(n)],
        dtype=float,
    )
    offs -= offs.mean(axis=0)
    return offs


def _diamond(n: int, spacing: float) -> np.ndarray:
    """One drone in the centre, the rest spread evenly around a ring."""
    if n == 1:
        return np.zeros((1, 2))
    offs = [(0.0, 0.0)]
    ring = n - 1
    ang = np.linspace(0.0, 2.0 * math.pi, ring, endpoint=False) + math.pi / ring
    for a in ang:
        offs.append((spacing * math.cos(a), spacing * math.sin(a)))
    return np.array(offs, dtype=float)


_GENERATORS = {
    "line": _line,
    "column": _column,
    "v": _v,
    "circle": _circle,
    "grid": _grid,
    "diamond": _diamond,
}

# Public list of supported formation names (used by the interface menu).
FORMATIONS: Tuple[str, ...] = tuple(sorted(_GENERATORS))


def compute_targets(
    formation: str,
    n: int,
    spacing: float,
    center: Sequence[float],
    altitude: float,
    yaw: float = 0.0,
) -> List[Position]:
    """Return a list of ``n`` (x, y, z) world/ENU targets for ``formation``.

    Args:
        formation: one of :data:`FORMATIONS`.
        n:         number of drones.
        spacing:   inter-drone spacing in metres.
        center:    (x, y[, z]) centre of the formation in the world frame.
        altitude:  target altitude (z), positive up.
        yaw:       heading the formation faces, radians (CCW from +X/east).

    Raises:
        ValueError: if ``formation`` is not recognised.
    """
    key = formation.lower().strip()
    if key not in _GENERATORS:
        raise ValueError(
            f"unknown formation '{formation}'; choose from {list(FORMATIONS)}"
        )
    if n <= 0:
        return []

    local = _GENERATORS[key](n, float(spacing))        # (n, 2) local XY
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    world = local @ rot.T                              # rotate into world frame
    world[:, 0] += float(center[0])
    world[:, 1] += float(center[1])

    z = float(altitude)
    return [(float(x), float(y), z) for x, y in world]
