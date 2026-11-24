import math
import random
import unittest

from benchmark import markdown, random_grid, run_benchmark, summarise


class RandomGridTests(unittest.TestCase):
    def test_start_and_goal_are_always_free(self):
        for seed in range(30):
            grid = random_grid(15, 0.9, random.Random(seed))
            self.assertEqual(grid[0][0], 0)
            self.assertEqual(grid[14][14], 0)

    def test_density_is_roughly_respected(self):
        grid = random_grid(60, 0.2, random.Random(0))
        share = sum(map(sum, grid)) / (60 * 60)
        self.assertAlmostEqual(share, 0.2, delta=0.04)

    def test_same_seed_gives_the_same_grid(self):
        self.assertEqual(random_grid(10, 0.3, random.Random(4)), random_grid(10, 0.3, random.Random(4)))


class BenchmarkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows = run_benchmark(densities=(0.10, 0.20), grids=4, size=20, seed=0, max_iters=800)

    def test_one_row_per_density(self):
        self.assertEqual([r["density"] for r in self.rows], [0.10, 0.20])

    def test_requested_number_of_solvable_grids_is_reached(self):
        self.assertTrue(all(r["solvable"] == 4 for r in self.rows))

    def test_astar_and_dijkstra_agree_so_astar_expands_fewer(self):
        for r in self.rows:
            self.assertLess(r["astar_expanded"], r["dijkstra_expanded"])

    def test_rrt_success_rate_is_a_share(self):
        for r in self.rows:
            self.assertTrue(0.0 <= r["rrt_success"] <= 1.0)

    def test_rrt_paths_are_not_far_shorter_than_optimal(self):
        for r in self.rows:
            if not math.isnan(r["rrt_length_ratio"]):
                self.assertGreater(r["rrt_length_ratio"], 0.95)

    def test_results_are_repeatable(self):
        again = run_benchmark(densities=(0.10, 0.20), grids=4, size=20, seed=0, max_iters=800)
        for first, second in zip(self.rows, again):
            for key in ("solvable", "unsolvable", "rrt_success", "rrt_length_ratio",
                        "astar_expanded", "dijkstra_expanded", "rrt_nodes"):
                self.assertEqual(first[key], second[key])  # timings are excluded: they vary

    def test_table_has_a_header_and_one_line_per_row(self):
        lines = markdown(self.rows).splitlines()
        self.assertEqual(len(lines), 2 + len(self.rows))
        self.assertIn("A* expanded", lines[0])

    def test_summarise_handles_no_solvable_grids(self):
        empty = {"solvable": 0, "unsolvable": 5, "rrt_ok": 0, "ratios": [], "astar": [],
                 "dijkstra": [], "rrt_nodes": [], "t_astar": [], "t_dijkstra": [], "t_rrt": []}
        row = summarise(0.3, empty)
        self.assertTrue(math.isnan(row["rrt_success"]))


if __name__ == "__main__":
    unittest.main()
