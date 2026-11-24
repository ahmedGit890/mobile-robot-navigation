"""Benchmark A*, Dijkstra and RRT on random cluttered grids.

For every grid we first ask A* whether a path exists. Grids with no path are
skipped (and counted) and new ones are generated until the requested number of
solvable grids is reached, so each planner is only judged on solvable problems.
Run `python benchmark.py` for the full run, or `--quick` for a fast smoke test.
Writes docs/benchmark.md.
"""
import argparse
import os
import random
import time

from robotnav.geometry import path_length
from robotnav.planner import search
from robotnav.rrt import rrt

SIZE = 40


def random_grid(size, density, rng):
    """Each cell is an obstacle with probability `density`; start and goal stay free."""
    grid = [[1 if rng.random() < density else 0 for _ in range(size)] for _ in range(size)]
    grid[0][0] = grid[size - 1][size - 1] = 0
    return grid


def timed(fn):
    t0 = time.perf_counter()
    value = fn()
    return value, (time.perf_counter() - t0) * 1000


def run_benchmark(densities=(0.10, 0.20, 0.30), grids=30, size=SIZE, seed=0, max_iters=2000):
    rows = []
    start, goal = (0, 0), (size - 1, size - 1)
    for density in densities:
        stats = {"solvable": 0, "unsolvable": 0, "rrt_ok": 0, "ratios": [],
                 "astar": [], "dijkstra": [], "rrt_nodes": [],
                 "t_astar": [], "t_dijkstra": [], "t_rrt": []}
        g = -1
        while stats["solvable"] < grids and g < grids * 50:  # keep going until enough solvable grids
            g += 1
            rng = random.Random(f"{seed}/{density}/{g}")
            grid = random_grid(size, density, rng)
            a, t_a = timed(lambda: search(grid, start, goal, use_heuristic=True))
            if a.path is None:
                stats["unsolvable"] += 1
                continue
            stats["solvable"] += 1
            d, t_d = timed(lambda: search(grid, start, goal, use_heuristic=False))
            r, t_r = timed(lambda: rrt(grid, start, goal, random.Random(f"rrt/{seed}/{density}/{g}"),
                                       max_iters=max_iters))
            stats["astar"].append(a.expanded)
            stats["dijkstra"].append(d.expanded)
            stats["rrt_nodes"].append(r.expanded)
            stats["t_astar"].append(t_a)
            stats["t_dijkstra"].append(t_d)
            stats["t_rrt"].append(t_r)
            if r.path is not None:
                stats["rrt_ok"] += 1
                stats["ratios"].append(path_length(r.path) / a.cost)
        rows.append(summarise(density, stats))
    return rows


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def summarise(density, s):
    solvable = s["solvable"]
    return {
        "density": density,
        "solvable": solvable,
        "unsolvable": s["unsolvable"],
        "rrt_success": s["rrt_ok"] / solvable if solvable else float("nan"),
        "rrt_length_ratio": mean(s["ratios"]),
        "astar_expanded": mean(s["astar"]),
        "dijkstra_expanded": mean(s["dijkstra"]),
        "rrt_nodes": mean(s["rrt_nodes"]),
        "astar_ms": mean(s["t_astar"]),
        "dijkstra_ms": mean(s["t_dijkstra"]),
        "rrt_ms": mean(s["t_rrt"]),
    }


def markdown(rows):
    head = ("| Obstacle density | Solvable grids | RRT success | RRT path vs optimal | "
            "A* expanded | Dijkstra expanded | RRT nodes | A* ms | Dijkstra ms | RRT ms |\n"
            "|---|---|---|---|---|---|---|---|---|---|")
    lines = [head]
    for r in rows:
        lines.append(
            f"| {r['density']:.0%} | {r['solvable']} ({r['unsolvable']} unsolvable skipped) | "
            f"{r['rrt_success']:.0%} | {r['rrt_length_ratio']:.2f}x | "
            f"{r['astar_expanded']:.0f} | {r['dijkstra_expanded']:.0f} | {r['rrt_nodes']:.0f} | "
            f"{r['astar_ms']:.1f} | {r['dijkstra_ms']:.1f} | {r['rrt_ms']:.1f} |")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true", help="5 grids per density, for a fast check")
    args = parser.parse_args()
    grids = 5 if args.quick else 30
    rows = run_benchmark(grids=grids)
    note = (f"> {SIZE}x{SIZE} grids, start at one corner and goal at the opposite corner, "
            f"{grids} solvable random grids per density, 8-connected moves. RRT: step 1.5 cells, "
            f"10% goal bias, at most 2000 iterations. Timings are pure Python on one machine "
            f"and only meaningful relative to each other.")
    text = f"{note}\n\n{markdown(rows)}\n"
    print(text)
    if not args.quick:
        os.makedirs("docs", exist_ok=True)
        with open("docs/benchmark.md", "w", encoding="utf-8") as f:
            f.write(text)


if __name__ == "__main__":
    main()
