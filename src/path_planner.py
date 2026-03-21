"""A* Path Planner for autonomous robot navigation."""
import heapq
import numpy as np


class AStarPlanner:
    """A* algorithm on a 2D occupancy grid."""

    def __init__(self, grid, resolution=0.05):
        self.grid = grid              # 2D numpy array: 0=free, 1=obstacle
        self.res  = resolution        # metres per cell
        self.rows, self.cols = grid.shape

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])   # Manhattan distance

    def plan(self, start, goal):
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self._reconstruct(came_from, current)

            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nb = (current[0]+dr, current[1]+dc)
                if not (0 <= nb[0] < self.rows and 0 <= nb[1] < self.cols):
                    continue
                if self.grid[nb] == 1:
                    continue
                step = 1.414 if dr and dc else 1.0
                tentative_g = g_score[current] + step
                if tentative_g < g_score.get(nb, float('inf')):
                    came_from[nb] = current
                    g_score[nb] = tentative_g
                    f = tentative_g + self.heuristic(nb, goal)
                    heapq.heappush(open_set, (f, nb))
        return []   # No path found

    def _reconstruct(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))


def cells_to_meters(path, resolution):
    return [(r * resolution, c * resolution) for r, c in path]
