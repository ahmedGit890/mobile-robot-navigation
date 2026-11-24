"""Small helpers shared by the planners, controller and demos."""
import math


def path_length(path):
    """Total length of a path (list of (row, col) points), in cell units."""
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def to_world(points, cell_size):
    """Grid points (row, col) -> world coordinates (x, y) in metres, at cell centres."""
    return [(c * cell_size + cell_size / 2, r * cell_size + cell_size / 2) for r, c in points]
