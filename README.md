# Mobile Robot Navigation (A* + Pure Pursuit)

A small 2D simulation of a differential-drive robot that plans a collision-free
path on an occupancy grid with A* and follows it with a pure-pursuit controller.

![demo](docs/demo.png)

## Run

```
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python -m unittest discover -s tests -t .
python demo.py            # writes docs/demo.png
```

## How it works

- `robotnav/planner.py`: A* on an 8-connected grid with an octile-distance
  heuristic, no corner cutting through diagonal obstacles.
- `robotnav/controller.py`: unicycle kinematics and a pure-pursuit follower
  that steers toward a lookahead point and slows down in sharp turns.
- `tests/`: unit tests for reachability, obstacle avoidance, blocked cells,
  corner cutting, angle wrapping and goal convergence.

## Roadmap (my next steps)

- [ ] Inflate obstacles by the robot radius so paths keep a safety margin
- [ ] Compare A* with Dijkstra and RRT: path length and nodes expanded
- [ ] Add a PID speed loop and measure cross-track error vs lookahead
- [ ] Port the planner and controller to ROS 2 nodes and run in Gazebo
