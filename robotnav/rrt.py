"""RRT: a sampling-based planner (Rapidly-exploring Random Tree).

Instead of searching every grid cell, RRT grows a tree from the start by
repeatedly picking a random point, finding the nearest tree node and stepping
toward the point if the move is collision free. It needs no map-wide search but
it is random and not optimal: the path is usually longer than A*'s, and it may
fail within the iteration limit even when a path exists.

Points are continuous (row, col) pairs where integer values are cell centres.
"""
import math

from .geometry import path_length
from .planner import PlanResult


def _is_free(grid, point):
    r, c = int(math.floor(point[0] + 0.5)), int(math.floor(point[1] + 0.5))
    return 0 <= r < len(grid) and 0 <= c < len(grid[0]) and not grid[r][c]


def collision_free(grid, a, b, resolution=0.25):
    """True if the straight segment a-b stays inside free cells (sampled every `resolution`)."""
    steps = max(1, math.ceil(math.dist(a, b) / resolution))
    for k in range(steps + 1):
        t = k / steps
        if not _is_free(grid, (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)):
            return False
    return True


def _steer(src, target, step):
    d = math.dist(src, target)
    if d <= step:
        return target
    t = step / d
    return (src[0] + (target[0] - src[0]) * t, src[1] + (target[1] - src[1]) * t)


def rrt(grid, start, goal, rng, max_iters=2000, step=1.5, goal_bias=0.1, goal_tol=1.5):
    """Grow a tree from `start` until it can reach `goal`, or give up after `max_iters`.

    rng: a random.Random, so results are repeatable.
    step: how far the tree grows toward a sample each iteration (cells).
    goal_bias: share of samples that are the goal itself (speeds up reaching it).
    goal_tol: the tree finishes once a node is this close to the goal with a clear line.
    """
    if grid[start[0]][start[1]] or grid[goal[0]][goal[1]]:
        return PlanResult(None, math.inf, 0)
    rows, cols = len(grid), len(grid[0])
    start_p, goal_p = (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))
    nodes, parent = [start_p], [-1]

    def result(goal_index):
        path, i = [], goal_index
        while i != -1:
            path.append(nodes[i])
            i = parent[i]
        path.reverse()
        edges = [(nodes[parent[i]], nodes[i]) for i in range(1, len(nodes))]
        return PlanResult(path, path_length(path), len(nodes), list(nodes), edges)

    for _ in range(max_iters):
        if rng.random() < goal_bias:
            sample = goal_p
        else:
            sample = (rng.uniform(0, rows - 1), rng.uniform(0, cols - 1))
        nearest = min(range(len(nodes)),
                      key=lambda i: (nodes[i][0] - sample[0]) ** 2 + (nodes[i][1] - sample[1]) ** 2)
        new = _steer(nodes[nearest], sample, step)
        if not collision_free(grid, nodes[nearest], new):
            continue
        nodes.append(new)
        parent.append(nearest)
        if math.dist(new, goal_p) <= goal_tol and collision_free(grid, new, goal_p):
            nodes.append(goal_p)
            parent.append(len(nodes) - 2)
            return result(len(nodes) - 1)
    edges = [(nodes[parent[i]], nodes[i]) for i in range(1, len(nodes))]
    return PlanResult(None, math.inf, len(nodes), list(nodes), edges)
