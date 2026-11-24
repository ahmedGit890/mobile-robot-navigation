import math
import random
import unittest

from robotnav.inflation import clearance, inflate
from robotnav.planner import search


def grid_with(cells, size=7):
    grid = [[0] * size for _ in range(size)]
    for r, c in cells:
        grid[r][c] = 1
    return grid


def blocked(grid):
    return {(r, c) for r, row in enumerate(grid) for c, v in enumerate(row) if v}


class InflateTests(unittest.TestCase):
    def test_radius_zero_changes_nothing(self):
        grid = grid_with([(3, 3)])
        self.assertEqual(inflate(grid, 0), grid)

    def test_radius_one_blocks_the_four_edge_neighbours_only(self):
        out = inflate(grid_with([(3, 3)]), 1.0)
        self.assertEqual(blocked(out), {(3, 3), (2, 3), (4, 3), (3, 2), (3, 4)})

    def test_radius_one_and_a_half_also_blocks_the_diagonals(self):
        out = inflate(grid_with([(3, 3)]), 1.5)
        self.assertEqual(len(blocked(out)), 9)

    def test_inflation_never_removes_an_obstacle(self):
        grid = grid_with([(1, 1), (5, 4), (3, 3)])
        self.assertTrue(blocked(grid) <= blocked(inflate(grid, 1.5)))

    def test_original_grid_is_not_modified(self):
        grid = grid_with([(3, 3)])
        before = [row[:] for row in grid]
        inflate(grid, 2.0)
        self.assertEqual(grid, before)

    def test_inflation_stops_at_the_map_edge(self):
        out = inflate(grid_with([(0, 0)]), 2.0)
        self.assertEqual(len(out), 7)
        self.assertTrue(all(len(row) == 7 for row in out))
        self.assertEqual(out[0][0], 1)

    def test_keep_free_cells_stay_free(self):
        out = inflate(grid_with([(3, 3)]), 1.0, keep_free=[(3, 4)])
        self.assertEqual(out[3][4], 0)
        self.assertEqual(out[2][3], 1)

    def test_bigger_radius_blocks_at_least_as_much(self):
        grid = grid_with([(3, 3), (1, 5)])
        self.assertTrue(blocked(inflate(grid, 1.0)) <= blocked(inflate(grid, 2.0)))

    def test_inflation_can_close_a_narrow_gap(self):
        # a wall with a one-cell gap: passable for a point robot, not for a fat one
        grid = [[0] * 7 for _ in range(7)]
        for r in range(7):
            if r != 3:
                grid[r][3] = 1
        self.assertIsNotNone(search(grid, (3, 0), (3, 6)).path)
        fat = inflate(grid, 1.0, keep_free=[(3, 0), (3, 6)])
        self.assertIsNone(search(fat, (3, 0), (3, 6)).path)


class ClearanceTests(unittest.TestCase):
    def test_no_obstacles_gives_infinity(self):
        self.assertEqual(clearance(grid_with([]), [(0, 0), (1, 1)]), math.inf)

    def test_adjacent_obstacle_gives_one(self):
        self.assertEqual(clearance(grid_with([(3, 3)]), [(3, 2)]), 1.0)

    def test_diagonal_neighbour_gives_root_two(self):
        self.assertAlmostEqual(clearance(grid_with([(3, 3)]), [(2, 2)]), math.sqrt(2))

    def test_clearance_is_the_minimum_over_the_path(self):
        grid = grid_with([(3, 3)])
        self.assertEqual(clearance(grid, [(0, 0), (3, 2), (6, 6)]), 1.0)


class InflatedPlanningTests(unittest.TestCase):
    def test_inflated_paths_keep_the_requested_clearance(self):
        # Sparse obstacles: with radius 2 each obstacle blocks 13 cells, so a dense map
        # would leave almost no solvable grids and the loop would check nothing.
        radius, size, checked = 2.0, 25, 0
        for seed in range(60):
            rng = random.Random(seed)
            grid = [[1 if rng.random() < 0.02 else 0 for _ in range(size)] for _ in range(size)]
            start, goal = (0, 0), (size - 1, size - 1)
            grid[0][0] = grid[size - 1][size - 1] = 0
            safe = inflate(grid, radius, keep_free=[start, goal])
            result = search(safe, start, goal)
            if result.path and len(result.path) > 2:
                checked += 1
                self.assertGreaterEqual(clearance(grid, result.path[1:-1]), radius - 1e-9)
        self.assertGreater(checked, 10)  # make sure the loop really exercised the property

    def test_a_safer_path_is_never_shorter(self):
        size, compared = 25, 0
        for seed in range(40):
            rng = random.Random(seed)
            grid = [[1 if rng.random() < 0.03 else 0 for _ in range(size)] for _ in range(size)]
            grid[0][0] = grid[size - 1][size - 1] = 0
            plain = search(grid, (0, 0), (size - 1, size - 1))
            safe = search(inflate(grid, 1.5, keep_free=[(0, 0), (size - 1, size - 1)]),
                          (0, 0), (size - 1, size - 1))
            if plain.path and safe.path:
                compared += 1
                self.assertGreaterEqual(safe.cost, plain.cost - 1e-9)
        self.assertGreater(compared, 10)  # make sure the loop really compared paths


if __name__ == "__main__":
    unittest.main()
