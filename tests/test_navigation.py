import math
import unittest

from robotnav.controller import PurePursuit, Robot, wrap_angle
from robotnav.planner import astar, heuristic


class PlannerTests(unittest.TestCase):
    def test_straight_line_on_empty_grid(self):
        grid = [[0] * 5 for _ in range(5)]
        path = astar(grid, (0, 0), (0, 4))
        self.assertEqual(path, [(0, i) for i in range(5)])

    def test_path_avoids_obstacles(self):
        grid = [[0] * 5 for _ in range(5)]
        for r in range(4):
            grid[r][2] = 1  # wall with a gap in the last row
        path = astar(grid, (0, 0), (0, 4))
        self.assertIsNotNone(path)
        self.assertTrue(all(grid[r][c] == 0 for r, c in path))
        self.assertIn((4, 2), path)

    def test_unreachable_returns_none(self):
        grid = [[0] * 5 for _ in range(5)]
        for r in range(5):
            grid[r][2] = 1
        self.assertIsNone(astar(grid, (0, 0), (0, 4)))

    def test_blocked_start_or_goal(self):
        grid = [[0, 1], [0, 0]]
        self.assertIsNone(astar(grid, (0, 1), (1, 1)))

    def test_no_corner_cutting(self):
        grid = [[0, 1], [1, 0]]
        self.assertIsNone(astar(grid, (0, 0), (1, 1)))

    def test_heuristic_is_admissible_on_diagonal(self):
        self.assertAlmostEqual(heuristic((0, 0), (3, 3)), 3 * math.sqrt(2))


class ControllerTests(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertAlmostEqual(abs(wrap_angle(2 * math.pi)), 0.0, places=6)
        self.assertAlmostEqual(wrap_angle(-3 * math.pi / 2), math.pi / 2)
        self.assertAlmostEqual(wrap_angle(math.pi / 2), math.pi / 2)

    def test_robot_drives_forward(self):
        r = Robot(0, 0, 0)
        r.step(1.0, 0.0, 1.0)
        self.assertAlmostEqual(r.x, 1.0)
        self.assertAlmostEqual(r.y, 0.0)

    def test_follower_reaches_goal(self):
        path = [(float(i), 0.0) for i in range(11)]
        robot, ctrl = Robot(0, 0, 0.5), PurePursuit(path)
        for _ in range(2000):
            if ctrl.done(robot):
                break
            robot.step(*ctrl.command(robot), 0.05)
        self.assertTrue(ctrl.done(robot))


if __name__ == "__main__":
    unittest.main()
