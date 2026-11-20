"""Plan with A*, follow with pure pursuit, save a plot to docs/demo.png."""
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from robotnav.controller import PurePursuit, Robot
from robotnav.planner import astar

CELL = 0.5  # metres per grid cell


def build_grid():
    grid = np.zeros((30, 40), dtype=int)
    grid[5:25, 12] = 1      # vertical wall with a gap at the top
    grid[0:5, 12] = 0
    grid[10, 20:38] = 1     # horizontal wall
    grid[18:28, 28] = 1
    return grid


def main():
    grid = build_grid()
    start, goal = (27, 2), (3, 37)
    cells = astar(grid.tolist(), start, goal)
    if cells is None:
        raise SystemExit("No path found")
    path = [(c * CELL + CELL / 2, r * CELL + CELL / 2) for r, c in cells]

    robot = Robot(path[0][0], path[0][1], 0.0)
    ctrl = PurePursuit(path)
    trail, dt = [(robot.x, robot.y)], 0.05
    for _ in range(4000):
        if ctrl.done(robot):
            break
        v, w = ctrl.command(robot)
        robot.step(v, w, dt)
        trail.append((robot.x, robot.y))
    reached = ctrl.done(robot)

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.imshow(grid, cmap="Greys", origin="upper",
              extent=(0, grid.shape[1] * CELL, grid.shape[0] * CELL, 0))
    ax.plot(*zip(*path), "b--", label="A* path")
    ax.plot(*zip(*trail), "r-", label="robot trail")
    ax.plot(*path[0], "go", label="start")
    ax.plot(*path[-1], "m*", markersize=14, label="goal")
    ax.set_title(f"A* + pure pursuit (goal reached: {reached})")
    ax.legend(loc="lower right")
    os.makedirs("docs", exist_ok=True)
    fig.savefig("docs/demo.png", dpi=120, bbox_inches="tight")
    length = sum(math.dist(a, b) for a, b in zip(trail, trail[1:]))
    print(f"goal reached: {reached}, steps: {len(trail)}, distance: {length:.1f} m")


if __name__ == "__main__":
    main()
