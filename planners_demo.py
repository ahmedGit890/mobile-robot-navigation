"""Two figures: three planners on one map (docs/planners.png), and the effect of
obstacle inflation on the planned path (docs/inflation.png)."""
import os
import random

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from demo import build_grid
from robotnav.geometry import path_length
from robotnav.inflation import clearance, inflate
from robotnav.planner import search
from robotnav.rrt import rrt

START, GOAL = (27, 2), (3, 37)
ROBOT_RADIUS = 2.0  # cells


def draw(ax, grid, result, title, inflated=None):
    rows, cols = len(grid), len(grid[0])
    shown = np.array(grid, dtype=float)
    if inflated is not None:
        shown = shown + (np.array(inflated, dtype=float) - shown) * 0.35  # inflated zone: light grey
    ax.imshow(shown, cmap="Greys", vmin=0, vmax=1, origin="upper",
              extent=(-0.5, cols - 0.5, rows - 0.5, -0.5))
    for p, q in result.edges:
        ax.plot([p[1], q[1]], [p[0], q[0]], color="tab:orange", lw=0.4)
    if result.explored and not result.edges:
        ax.scatter([c for _, c in result.explored], [r for r, _ in result.explored],
                   s=4, color="tab:blue", alpha=0.35)
    if result.path:
        ax.plot([c for _, c in result.path], [r for r, _ in result.path], "r-", lw=2)
    ax.plot(START[1], START[0], "go")
    ax.plot(GOAL[1], GOAL[0], "m*", markersize=12)
    ax.set_title(title, fontsize=9)
    ax.set_xticks([])
    ax.set_yticks([])


def planners_figure(grid):
    astar = search(grid, START, GOAL, use_heuristic=True)
    dijk = search(grid, START, GOAL, use_heuristic=False)
    tree = rrt(grid, START, GOAL, random.Random(1), max_iters=3000)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    draw(axes[0], grid, astar, f"A*: {astar.expanded} cells expanded, path {astar.cost:.1f}")
    draw(axes[1], grid, dijk, f"Dijkstra: {dijk.expanded} cells expanded, path {dijk.cost:.1f}")
    rrt_title = (f"RRT: {tree.expanded} nodes, path {tree.cost:.1f}" if tree.path
                 else f"RRT: no path in {tree.expanded} nodes")
    draw(axes[2], grid, tree, rrt_title)
    fig.suptitle("Same map, three planners (blue = expanded cells, orange = RRT tree, red = path)")
    os.makedirs("docs", exist_ok=True)
    fig.savefig("docs/planners.png", dpi=110, bbox_inches="tight")
    print(f"A*       expanded {astar.expanded:5d}  length {astar.cost:.1f}")
    print(f"Dijkstra expanded {dijk.expanded:5d}  length {dijk.cost:.1f}")
    print(f"RRT      nodes    {tree.expanded:5d}  length {tree.cost:.1f}")


def inflation_figure(grid):
    plain = search(grid, START, GOAL)
    inflated_grid = inflate(grid, ROBOT_RADIUS, keep_free=[START, GOAL])
    safe = search(inflated_grid, START, GOAL)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax, result, shown, label in ((axes[0], plain, None, "point robot"),
                                     (axes[1], safe, inflated_grid, f"robot radius {ROBOT_RADIUS:g} cells")):
        if result.path:
            margin = clearance(grid, result.path[1:-1])  # judge the path itself, not start/goal
            title = f"{label}: path {result.cost:.1f}, closest approach {margin:.1f} cells"
        else:
            title = f"{label}: no path"
        draw(ax, grid, result, title, inflated=shown)
    fig.suptitle("Obstacle inflation keeps the robot's body off the walls (light grey = inflated zone)")
    fig.savefig("docs/inflation.png", dpi=110, bbox_inches="tight")
    for label, result in (("point robot", plain), ("inflated", safe)):
        if result.path:
            print(f"{label:12s} length {result.cost:.1f}  "
                  f"closest approach {clearance(grid, result.path[1:-1]):.2f} cells")
        else:
            print(f"{label:12s} no path")


def main():
    grid = build_grid().tolist()
    planners_figure(grid)
    inflation_figure(grid)


if __name__ == "__main__":
    main()
