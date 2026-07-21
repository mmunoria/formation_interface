"""Prioritized 2-D grid path planning for the drone swarm.

No ROS imports on purpose (same pattern as ``formations.py``): this module is
unit-tested standalone and imported by the formation node.

Model:
  * The indoor flight space is a rectangle (:class:`Arena`), discretised into
    square cells of ``cell_size`` metres.
  * Planning is purely 2-D - altitude is constant and handled by the caller.
  * **Prioritized planning**: drones are planned one at a time in index order.
    For drone *k*, the obstacles are every other drone's current position plus
    the full paths already planned for drones ``0..k-1``, all inflated by
    ``safety_radius``.
  * Paths come out of A* (8-connected, octile heuristic) and are then
    simplified with line-of-sight pruning, so a straight shot is exactly
    ``[start, goal]``.

Raises :class:`PlanningError` (with the offending drone's display ID in the
message) when a goal is outside the arena, sits inside another drone's safety
radius, or no collision-free path exists.
"""

import heapq
import math
from typing import List, Sequence, Set, Tuple

XY = Tuple[float, float]

_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
               (0, 1), (1, -1), (1, 0), (1, 1)]


class PlanningError(ValueError):
    """A drone's path could not be planned; the message says which and why."""


class Arena:
    """Axis-aligned rectangular flight space in the map/world frame."""

    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float):
        if max_x <= min_x or max_y <= min_y:
            raise ValueError("arena max bounds must exceed min bounds")
        self.min_x, self.max_x = float(min_x), float(max_x)
        self.min_y, self.max_y = float(min_y), float(max_y)

    def contains(self, x: float, y: float) -> bool:
        return (self.min_x <= x <= self.max_x
                and self.min_y <= y <= self.max_y)

    def clamp(self, x: float, y: float) -> XY:
        return (min(max(x, self.min_x), self.max_x),
                min(max(y, self.min_y), self.max_y))


class _Grid:
    """Discretisation of an arena into square cells."""

    def __init__(self, arena: Arena, cell: float):
        if cell <= 0:
            raise ValueError("cell_size must be positive")
        self.arena = arena
        self.cell = float(cell)
        self.nx = max(1, int(math.ceil((arena.max_x - arena.min_x) / cell)))
        self.ny = max(1, int(math.ceil((arena.max_y - arena.min_y) / cell)))

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        i = int((x - self.arena.min_x) / self.cell)
        j = int((y - self.arena.min_y) / self.cell)
        return (min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1))

    def center(self, c: Tuple[int, int]) -> XY:
        return (self.arena.min_x + (c[0] + 0.5) * self.cell,
                self.arena.min_y + (c[1] + 0.5) * self.cell)

    def cells_near(self, x: float, y: float, radius: float):
        i0, j0 = self.cell_of(x - radius, y - radius)
        i1, j1 = self.cell_of(x + radius, y + radius)
        r2 = radius * radius
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                cx, cy = self.center((i, j))
                if (cx - x) ** 2 + (cy - y) ** 2 <= r2:
                    yield (i, j)


def _inflate(grid: _Grid, points: Sequence[XY], radius: float) -> Set[Tuple[int, int]]:
    blocked: Set[Tuple[int, int]] = set()
    for (ox, oy) in points:
        blocked.update(grid.cells_near(ox, oy, radius))
    return blocked


def _octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (_SQRT2 - 1.0) * min(dx, dy)


def _astar(blocked, nx, ny, start, goal):
    """8-connected A* on the grid. Returns a cell list or None."""
    if start == goal:
        return [start]
    open_heap = [(_octile(start, goal), 0.0, start)]
    best_g = {start: 0.0}
    parent = {}
    while open_heap:
        f, g, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            return path[::-1]
        if g > best_g.get(cur, math.inf):
            continue
        for di, dj in _NEIGHBOURS:
            nxt = (cur[0] + di, cur[1] + dj)
            if not (0 <= nxt[0] < nx and 0 <= nxt[1] < ny):
                continue
            if nxt in blocked:
                continue
            ng = g + (_SQRT2 if di and dj else 1.0)
            if ng < best_g.get(nxt, math.inf):
                best_g[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(open_heap, (ng + _octile(nxt, goal), ng, nxt))
    return None


def _line_free(grid: _Grid, blocked, a: XY, b: XY) -> bool:
    """True if the straight segment a->b crosses no blocked cell."""
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    steps = max(1, int(dist / (grid.cell * 0.5)))
    for s in range(steps + 1):
        t = s / steps
        x = a[0] + t * (b[0] - a[0])
        y = a[1] + t * (b[1] - a[1])
        if grid.cell_of(x, y) in blocked:
            return False
    return True


def _simplify(points: List[XY], grid: _Grid, blocked) -> List[XY]:
    """Line-of-sight pruning: keep the farthest visible waypoint each hop."""
    if len(points) <= 2:
        return points
    out = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not _line_free(grid, blocked, points[i], points[j]):
            j -= 1
        out.append(points[j])
        i = j
    return out


def _densify(path: Sequence[XY], step: float) -> List[XY]:
    """Sample a waypoint path at ``step`` intervals (for use as obstacles)."""
    out: List[XY] = [tuple(path[0])]
    for a, b in zip(path, path[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(dist / step))
        for s in range(1, n + 1):
            t = s / n
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


def _plan_one(grid: _Grid, blocked, start: XY, goal: XY) -> List[XY]:
    local = set(blocked)
    # Escape hatch: a neighbour parked close by may have inflated over our own
    # start cell - carve out the immediate vicinity so the drone can move off.
    for c in grid.cells_near(start[0], start[1], grid.cell * 1.6):
        local.discard(c)

    goal_cell = grid.cell_of(*goal)
    if goal_cell in local:
        raise PlanningError(
            "goal is within the safety radius of another drone or its path")

    cells = _astar(local, grid.nx, grid.ny, grid.cell_of(*start), goal_cell)
    if cells is None:
        raise PlanningError("no collision-free path exists in the arena")

    points = [start] + [grid.center(c) for c in cells[1:-1]] + [goal]
    return _simplify(points, grid, local)


def plan_paths(
    starts: Sequence[XY],
    goals: Sequence[XY],
    arena: Arena,
    cell_size: float = 0.1,
    safety_radius: float = 0.4,
    static_obstacles: Sequence[XY] = (),
) -> Tuple[List[List[XY]], List[int]]:
    """``static_obstacles`` are other drones that exist but aren't part of
    this planning call (e.g. holding a target from an earlier goal). They're
    inflated as hard obstacles for every drone planned here, in both the
    normal and the path-crossing-allowed fallback attempt - unlike another
    *planned* drone's path, we can't ask a drone outside this goal to move.
    """
    if len(starts) != len(goals):
        raise ValueError("starts and goals must have the same length")
    n = len(starts)
    for k, (gx, gy) in enumerate(goals):
        if not arena.contains(gx, gy):
            raise PlanningError(
                f"drone {k + 1}: goal ({gx:.2f}, {gy:.2f}) is outside the arena")

    grid = _Grid(arena, cell_size)
    starts = [arena.clamp(x, y) for (x, y) in starts]
    static_obstacles = list(static_obstacles)

    paths: List[List[XY]] = []
    crossings: List[int] = []
    for k in range(n):
        endpoint_obs = [starts[m] for m in range(n) if m != k]
        endpoint_obs += [goals[m] for m in range(k)]
        endpoint_obs += static_obstacles
        path_obs = [pt for m in range(k) for pt in _densify(paths[m], cell_size)]

        try:
            blocked = _inflate(grid, endpoint_obs + path_obs, safety_radius)
            paths.append(_plan_one(grid, blocked, starts[k], goals[k]))
        except PlanningError:
            try:
                blocked = _inflate(grid, endpoint_obs, safety_radius)
                paths.append(_plan_one(grid, blocked, starts[k], goals[k]))
                crossings.append(k + 1)
            except PlanningError as exc:
                raise PlanningError(f"drone {k + 1}: {exc}") from None
    return paths, crossings
