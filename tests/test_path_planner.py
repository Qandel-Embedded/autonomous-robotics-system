"""Unit tests for the A* path planner."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
import pytest
from path_planner import AStarPlanner, cells_to_meters


def test_clear_grid_finds_path():
    grid = np.zeros((5, 5), dtype=int)
    planner = AStarPlanner(grid)
    path = planner.plan((0, 0), (4, 4))
    assert len(path) > 0
    assert path[0]  == (0, 0)
    assert path[-1] == (4, 4)


def test_blocked_start_raises():
    grid = np.ones((5, 5), dtype=int)
    planner = AStarPlanner(grid)
    path = planner.plan((0, 0), (4, 4))
    assert path == []


def test_cells_to_meters():
    path = [(0, 0), (1, 0), (2, 0)]
    result = cells_to_meters(path, resolution=0.05)
    assert result[2] == (0.10, 0.0)


def test_kalman_filter_converges():
    from sensor_fusion import KalmanFilter
    kf = KalmanFilter()
    estimates = [kf.update(kf.predict() + m) for m in [1.0]*20]
    assert abs(estimates[-1] - 1.0) < 0.5
