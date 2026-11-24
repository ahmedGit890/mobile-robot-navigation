> 40x40 grids, start at one corner and goal at the opposite corner, 30 solvable random grids per density, 8-connected moves. RRT: step 1.5 cells, 10% goal bias, at most 2000 iterations. Timings are pure Python on one machine and only meaningful relative to each other.

| Obstacle density | Solvable grids | RRT success | RRT path vs optimal | A* expanded | Dijkstra expanded | RRT nodes | A* ms | Dijkstra ms | RRT ms |
|---|---|---|---|---|---|---|---|---|---|
| 10% | 30 (2 unsolvable skipped) | 90% | 1.14x | 301 | 1440 | 419 | 1.1 | 4.0 | 39.4 |
| 20% | 30 (8 unsolvable skipped) | 67% | 1.14x | 434 | 1278 | 676 | 1.3 | 3.1 | 86.3 |
| 30% | 30 (28 unsolvable skipped) | 37% | 1.08x | 519 | 1083 | 589 | 1.4 | 2.4 | 80.5 |
