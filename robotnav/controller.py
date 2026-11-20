"""Differential-drive robot model and a pure-pursuit path follower."""
import math
from dataclasses import dataclass


@dataclass
class Robot:
    x: float
    y: float
    theta: float  # heading in radians

    def step(self, v, omega, dt):
        """Unicycle kinematics: forward speed v (m/s), turn rate omega (rad/s)."""
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = wrap_angle(self.theta + omega * dt)


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PurePursuit:
    """Steer toward a lookahead point on the path.

    Tuning notes (try changing these and watch the plot):
      lookahead  - larger = smoother but cuts corners; smaller = tighter but wobbly
      v_max      - cruising speed
      k_turn     - how aggressively heading error becomes turn rate
    """

    def __init__(self, path, lookahead=0.8, v_max=1.0, k_turn=2.0, goal_tol=0.15):
        self.path = path  # list of (x, y) in metres
        self.lookahead = lookahead
        self.v_max = v_max
        self.k_turn = k_turn
        self.goal_tol = goal_tol
        self._idx = 0

    def done(self, robot):
        gx, gy = self.path[-1]
        return math.hypot(gx - robot.x, gy - robot.y) < self.goal_tol

    def command(self, robot):
        # Advance to the first path point that is at least `lookahead` away.
        while self._idx < len(self.path) - 1:
            px, py = self.path[self._idx]
            if math.hypot(px - robot.x, py - robot.y) >= self.lookahead:
                break
            self._idx += 1
        tx, ty = self.path[self._idx]
        heading_err = wrap_angle(math.atan2(ty - robot.y, tx - robot.x) - robot.theta)
        omega = self.k_turn * heading_err
        # Slow down in sharp turns so the robot does not overshoot.
        v = self.v_max * max(0.2, math.cos(heading_err))
        return v, omega
