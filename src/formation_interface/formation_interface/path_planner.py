"""Grid-based A* path planner - no ROS dependencies on purpose.

Plans a path for one drone across a walled rectangle, treating every other
active drone's position as an inflated point obstacle. With zero obstacles
(only one drone active) this reduces to routing straight across the empty
grid - "obstacle-free except the walls" falls out of the algorithm rather
than needing a special case.
"""

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

Point2D = Tuple[float, float]
Cell = Tuple[int, int]
Bounds = Tuple[float, float, float, float]   # xmin, xmax, ymin, ymax

_NEIGHBOR_OFFSETS = [
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1), (0, 1),
    (1, -1), (1, 0), (1, 1),
]


class GridAStarPlanner:
    """8-connected A* over a rectangular grid.

    Args:
        bounds:        (xmin, xmax, ymin, ymax) of the flight area, metres.
        cell_size:      grid resolution, metres.
        safety_radius: obstacles block every cell within this distance, metres.
    """

    def __init__(self, bounds: Bounds, cell_size: float = 0.25,
                 safety_radius: float = 0.5):
        self.xmin, self.xmax, self.ymin, self.ymax = (float(b) for b in bounds)
        if self.xmax <= self.xmin or self.ymax <= self.ymin:
            raise ValueError(f"invalid bounds {bounds}")
        self.cell_size = float(cell_size)
        self.safety_radius = float(safety_radius)
        self.cols = max(1, math.ceil((self.xmax - self.xmin) / self.cell_size))
        self.rows = max(1, math.ceil((self.ymax - self.ymin) / self.cell_size))

    def plan(
        self,
        start: Point2D,
        goal: Point2D,
        obstacles: Sequence[Point2D] = (),
    ) -> Optional[List[Point2D]]:
        """Return a world-frame waypoint list from ``start`` to ``goal``.

        Returns ``None`` if no path exists (goal boxed in, or start/goal
        itself sits inside an inflated obstacle).

        Raises:
            ValueError: if ``start`` or ``goal`` is outside ``bounds``.
        """
        if not self._in_bounds(start):
            raise ValueError(f"start {start} is outside map bounds")
        if not self._in_bounds(goal):
            raise ValueError(f"goal {goal} is outside map bounds")

        blocked = self._blocked_cells(obstacles)
        start_cell, goal_cell = self._to_cell(start), self._to_cell(goal)
        if start_cell in blocked or goal_cell in blocked:
            return None

        cell_path = self._astar(start_cell, goal_cell, blocked)
        if cell_path is None:
            return None

        world_path = [tuple(start)]
        world_path.extend(self._to_world(c) for c in cell_path[1:-1])
        world_path.append(tuple(goal))
        return world_path

    # ------------------------------ grid <-> world --------------------------
    def _in_bounds(self, p: Point2D) -> bool:
        x, y = p
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax

    def _to_cell(self, p: Point2D) -> Cell:
        x, y = p
        col = min(max(int((x - self.xmin) / self.cell_size), 0), self.cols - 1)
        row = min(max(int((y - self.ymin) / self.cell_size), 0), self.rows - 1)
        return col, row

    def _to_world(self, cell: Cell) -> Point2D:
        col, row = cell
        return (self.xmin + (col + 0.5) * self.cell_size,
                self.ymin + (row + 0.5) * self.cell_size)

    # ------------------------------ obstacles --------------------------------
    def _blocked_cells(self, obstacles: Sequence[Point2D]):
        blocked = set()
        reach = max(0, math.ceil(self.safety_radius / self.cell_size))
        for ox, oy in obstacles:
            ocol, orow = self._to_cell((ox, oy))
            for dc in range(-reach, reach + 1):
                for dr in range(-reach, reach + 1):
                    c, r = ocol + dc, orow + dr
                    if not (0 <= c < self.cols and 0 <= r < self.rows):
                        continue
                    cx, cy = self._to_world((c, r))
                    if math.hypot(cx - ox, cy - oy) <= self.safety_radius:
                        blocked.add((c, r))
        return blocked

    # ------------------------------ search -----------------------------------
    def _astar(self, start: Cell, goal: Cell, blocked) -> Optional[List[Cell]]:
        def heuristic(a: Cell, b: Cell) -> float:
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_heap = [(heuristic(start, goal), 0.0, start)]
        came_from: Dict[Cell, Cell] = {}
        g_score = {start: 0.0}
        visited = set()

        while open_heap:
            _, g, current = heapq.heappop(open_heap)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                return self._reconstruct(came_from, current)

            for dc, dr in _NEIGHBOR_OFFSETS:
                nb = (current[0] + dc, current[1] + dr)
                if not (0 <= nb[0] < self.cols and 0 <= nb[1] < self.rows):
                    continue
                if nb in blocked or nb in visited:
                    continue
                step_g = g + math.hypot(dc, dr)
                if step_g < g_score.get(nb, math.inf):
                    g_score[nb] = step_g
                    came_from[nb] = current
                    heapq.heappush(open_heap, (step_g + heuristic(nb, goal), step_g, nb))

        return None

    @staticmethod
    def _reconstruct(came_from: Dict[Cell, Cell], current: Cell) -> List[Cell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
