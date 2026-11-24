"""Obstacle inflation: keep the robot's body away from walls.

A grid planner treats the robot as a single point. A real robot has a radius, so
a path that squeezes past a wall corner would scrape it. Inflation marks every
cell within the robot's radius of an obstacle as blocked, so the point planner
automatically keeps a safe margin.
"""
import math


def inflate(grid, radius, keep_free=()):
    """Return a copy of `grid` with obstacles grown by `radius` cells.

    radius: in cell units (1.0 blocks the four edge neighbours, 1.5 also the diagonals).
    keep_free: cells (such as the start and goal) that must stay free even if they
        fall inside an inflated zone; otherwise the planner could not start or finish.
    """
    rows, cols = len(grid), len(grid[0])
    out = [row[:] for row in grid]
    reach = math.ceil(radius)
    limit = radius * radius
    for r in range(rows):
        for c in range(cols):
            if not grid[r][c]:
                continue
            for dr in range(-reach, reach + 1):
                for dc in range(-reach, reach + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols and dr * dr + dc * dc <= limit:
                        out[nr][nc] = 1
    for r, c in keep_free:
        out[r][c] = 0
    return out


def clearance(grid, path):
    """Smallest distance (in cells) from any path point to the nearest obstacle cell.

    Returns infinity if the grid has no obstacles. Measured between cell centres.
    """
    obstacles = [(r, c) for r, row in enumerate(grid) for c, v in enumerate(row) if v]
    if not obstacles:
        return math.inf
    return min(min(math.dist(p, o) for o in obstacles) for p in path)
