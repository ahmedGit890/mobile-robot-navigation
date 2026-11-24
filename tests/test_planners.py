import math
import random
import unittest

from robotnav.geometry import path_length, to_world
from robotnav.planner import astar, dijkstra, search


def random_grid(size, density, seed):
    rng = random.Random(seed)
    grid = [[1 if rng.random() < density else 0 for _ in range(size)] for _ in range(size)]
    grid[0][0] = grid[size - 1][size - 1] = 0
    return grid


class SearchResultTests(unittest.TestCase):
    def test_result_describes_the_search(self):
        grid = [[0] * 6 for _ in range(6)]
        result = search(grid, (0, 0), (5, 5))
        self.assertEqual(result.path[0], (0, 0))
        self.assertEqual(result.path[-1], (5, 5))
        self.assertAlmostEqual(result.cost, 5 * math.sqrt(2))
        self.assertEqual(result.expanded, len(result.explored))
        self.assertGreater(result.expanded, 0)

    def test_unreachable_goal_reports_no_path_and_infinite_cost(self):
        grid = [[0, 1, 0], [0, 1, 0], [0, 1, 0]]
        result = search(grid, (0, 0), (0, 2))
        self.assertIsNone(result.path)
        self.assertEqual(result.cost, math.inf)

    def test_blocked_start_expands_nothing(self):
        result = search([[1, 0], [0, 0]], (0, 0), (1, 1))
        self.assertIsNone(result.path)
        self.assertEqual(result.expanded, 0)

    def test_astar_wrapper_returns_the_same_path_as_search(self):
        grid = random_grid(15, 0.15, 3)
        self.assertEqual(astar(grid, (0, 0), (14, 14)), search(grid, (0, 0), (14, 14)).path)


class AstarVersusDijkstraTests(unittest.TestCase):
    def solvable_grids(self, count=40, size=20, density=0.15):
        found, seed = [], 0
        while len(found) < count:
            grid = random_grid(size, density, seed)
            seed += 1
            if search(grid, (0, 0), (size - 1, size - 1)).path:
                found.append(grid)
        return found

    def test_both_find_a_path_of_the_same_optimal_length(self):
        for grid in self.solvable_grids():
            a = search(grid, (0, 0), (19, 19), use_heuristic=True)
            d = search(grid, (0, 0), (19, 19), use_heuristic=False)
            self.assertAlmostEqual(a.cost, d.cost, places=9)

    def test_astar_expands_fewer_cells_than_dijkstra_on_average(self):
        a_total = d_total = 0
        for grid in self.solvable_grids():
            a_total += search(grid, (0, 0), (19, 19), use_heuristic=True).expanded
            d_total += search(grid, (0, 0), (19, 19), use_heuristic=False).expanded
        self.assertLess(a_total, d_total)

    def test_dijkstra_agrees_with_astar_about_reachability(self):
        for seed in range(40):
            grid = random_grid(12, 0.35, seed)
            self.assertEqual(astar(grid, (0, 0), (11, 11)) is None,
                             dijkstra(grid, (0, 0), (11, 11)) is None)

    def test_dijkstra_wrapper_returns_a_path(self):
        grid = [[0] * 5 for _ in range(5)]
        self.assertEqual(dijkstra(grid, (0, 0), (0, 4)), [(0, i) for i in range(5)])

    def test_the_path_cost_matches_its_geometric_length(self):
        grid = random_grid(20, 0.15, 7)
        result = search(grid, (0, 0), (19, 19))
        if result.path:
            self.assertAlmostEqual(result.cost, path_length(result.path), places=9)


class GeometryTests(unittest.TestCase):
    def test_path_length_of_a_straight_and_a_diagonal_move(self):
        self.assertAlmostEqual(path_length([(0, 0), (0, 3)]), 3.0)
        self.assertAlmostEqual(path_length([(0, 0), (1, 1)]), math.sqrt(2))

    def test_path_length_of_a_single_point_is_zero(self):
        self.assertEqual(path_length([(2, 2)]), 0)

    def test_to_world_uses_cell_centres_and_swaps_row_col_to_xy(self):
        self.assertEqual(to_world([(2, 4)], 0.5), [(4 * 0.5 + 0.25, 2 * 0.5 + 0.25)])


if __name__ == "__main__":
    unittest.main()
