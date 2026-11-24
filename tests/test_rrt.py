import math
import random
import unittest

from robotnav.geometry import path_length
from robotnav.planner import search
from robotnav.rrt import collision_free, rrt


def empty(size=20):
    return [[0] * size for _ in range(size)]


def walled(size=20):
    grid = empty(size)
    for r in range(size):
        grid[r][size // 2] = 1  # a full wall: no way across
    return grid


class CollisionTests(unittest.TestCase):
    def test_free_segment(self):
        self.assertTrue(collision_free(empty(), (0.0, 0.0), (10.0, 10.0)))

    def test_segment_through_an_obstacle_is_blocked(self):
        grid = empty()
        grid[5][5] = 1
        self.assertFalse(collision_free(grid, (0.0, 0.0), (10.0, 10.0)))

    def test_segment_leaving_the_map_is_blocked(self):
        self.assertFalse(collision_free(empty(), (0.0, 0.0), (0.0, 25.0)))

    def test_zero_length_segment_in_a_free_cell(self):
        self.assertTrue(collision_free(empty(), (3.0, 3.0), (3.0, 3.0)))


class RRTTests(unittest.TestCase):
    def test_finds_a_path_on_an_empty_map(self):
        result = rrt(empty(), (0, 0), (19, 19), random.Random(0))
        self.assertIsNotNone(result.path)
        self.assertEqual(result.path[0], (0.0, 0.0))
        self.assertEqual(result.path[-1], (19.0, 19.0))

    def test_every_segment_of_the_path_is_collision_free(self):
        grid = empty()
        for r in range(2, 18):
            grid[r][10] = 1  # a wall with gaps at the top and bottom
        result = rrt(grid, (10, 2), (10, 18), random.Random(2), max_iters=4000)
        self.assertIsNotNone(result.path)
        for a, b in zip(result.path, result.path[1:]):
            self.assertTrue(collision_free(grid, a, b, resolution=0.1))

    def test_same_seed_gives_the_same_path(self):
        a = rrt(empty(), (0, 0), (19, 19), random.Random(5))
        b = rrt(empty(), (0, 0), (19, 19), random.Random(5))
        self.assertEqual(a.path, b.path)

    def test_different_seeds_explore_differently(self):
        a = rrt(empty(), (0, 0), (19, 19), random.Random(5))
        b = rrt(empty(), (0, 0), (19, 19), random.Random(6))
        self.assertNotEqual(a.edges, b.edges)

    def test_no_path_through_a_solid_wall(self):
        result = rrt(walled(), (10, 2), (10, 18), random.Random(0), max_iters=300)
        self.assertIsNone(result.path)
        self.assertEqual(result.cost, math.inf)

    def test_stops_at_the_iteration_limit(self):
        result = rrt(walled(), (10, 2), (10, 18), random.Random(0), max_iters=50)
        self.assertLessEqual(result.expanded, 51)  # start node plus at most one node per iteration

    def test_blocked_start_or_goal_fails_immediately(self):
        grid = empty()
        grid[0][0] = 1
        self.assertIsNone(rrt(grid, (0, 0), (5, 5), random.Random(0)).path)

    def test_path_is_never_shorter_than_the_straight_line(self):
        result = rrt(empty(), (0, 0), (19, 19), random.Random(1))
        self.assertGreaterEqual(result.cost, math.dist((0, 0), (19, 19)) - 1e-9)
        self.assertAlmostEqual(result.cost, path_length(result.path))

    def test_rrt_is_never_better_than_the_optimal_grid_path_by_more_than_rounding(self):
        # Grid A* is optimal on the 8-connected grid, but RRT moves in free space, so it may
        # beat it slightly; it should never be wildly shorter.
        grid = empty()
        optimal = search(grid, (0, 0), (19, 19)).cost
        for seed in range(10):
            result = rrt(grid, (0, 0), (19, 19), random.Random(seed))
            self.assertGreaterEqual(result.cost, optimal * 0.95)

    def test_tree_edges_connect_existing_nodes(self):
        result = rrt(empty(), (0, 0), (19, 19), random.Random(3))
        nodes = set(result.explored)
        for p, q in result.edges:
            self.assertIn(p, nodes)
            self.assertIn(q, nodes)


if __name__ == "__main__":
    unittest.main()
