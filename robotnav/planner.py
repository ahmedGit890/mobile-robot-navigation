"""Grid path planning: A* and Dijkstra on a 2D occupancy grid.

Grid convention: grid[row][col] == 1 means obstacle, 0 means free.
Cells are (row, col) tuples. Movement is 8-connected.

Dijkstra is A* with no heuristic: it expands cells in order of distance from the
start only. Both find the cheapest path; A* usually expands far fewer cells
because the heuristic steers it toward the goal.
"""
import heapq
import math
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PlanResult:
    """What a planner returns, so different planners can be compared the same way."""
    path: Optional[list]          # points from start to goal, or None if no path was found
    cost: float                   # path length in cell units (inf if no path)
    expanded: int                 # cells expanded (grid search) or tree nodes (RRT)
    explored: list = field(default_factory=list)  # expanded cells / tree nodes, in order
    edges: list = field(default_factory=list)     # tree edges (RRT only), for plotting


def heuristic(a, b):
    """Octile distance: the exact cost on an empty 8-connected grid."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)


def neighbors(grid, cell):
    rows, cols = len(grid), len(grid[0])
    r, c = cell
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or grid[nr][nc]:
                continue
            # Forbid cutting corners between two diagonal obstacles.
            if dr and dc and (grid[r + dr][c] or grid[r][c + dc]):
                continue
            yield (nr, nc), math.hypot(dr, dc)


def _no_heuristic(a, b):
    return 0.0


def search(grid, start, goal, use_heuristic=True):
    """Best-first search. With the heuristic it is A*; without it, Dijkstra."""
    if grid[start[0]][start[1]] or grid[goal[0]][goal[1]]:
        return PlanResult(None, math.inf, 0)
    h = heuristic if use_heuristic else _no_heuristic
    open_heap = [(h(start, goal), 0.0, start)]
    came_from = {}
    best_cost = {start: 0.0}
    explored = []
    while open_heap:
        _, cost, current = heapq.heappop(open_heap)
        if cost > best_cost.get(current, math.inf):
            continue  # stale heap entry
        explored.append(current)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return PlanResult(path[::-1], cost, len(explored), explored)
        for nxt, step in neighbors(grid, current):
            new_cost = cost + step
            if new_cost < best_cost.get(nxt, math.inf):
                best_cost[nxt] = new_cost
                came_from[nxt] = current
                heapq.heappush(open_heap, (new_cost + h(nxt, goal), new_cost, nxt))
    return PlanResult(None, math.inf, len(explored), explored)


def astar(grid, start, goal):
    """Return the list of cells from start to goal, or None if unreachable."""
    return search(grid, start, goal, use_heuristic=True).path


def dijkstra(grid, start, goal):
    """Same as astar, but without a heuristic."""
    return search(grid, start, goal, use_heuristic=False).path
