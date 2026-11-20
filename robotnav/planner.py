"""A* path planning on a 2D occupancy grid.

Grid convention: grid[row][col] == 1 means obstacle, 0 means free.
Cells are (row, col) tuples. Movement is 8-connected.
"""
import heapq
import math


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


def astar(grid, start, goal):
    """Return the list of cells from start to goal, or None if unreachable."""
    if grid[start[0]][start[1]] or grid[goal[0]][goal[1]]:
        return None
    open_heap = [(heuristic(start, goal), 0.0, start)]
    came_from = {}
    best_cost = {start: 0.0}
    while open_heap:
        _, cost, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        if cost > best_cost.get(current, math.inf):
            continue  # stale heap entry
        for nxt, step in neighbors(grid, current):
            new_cost = cost + step
            if new_cost < best_cost.get(nxt, math.inf):
                best_cost[nxt] = new_cost
                came_from[nxt] = current
                heapq.heappush(
                    open_heap, (new_cost + heuristic(nxt, goal), new_cost, nxt)
                )
    return None
