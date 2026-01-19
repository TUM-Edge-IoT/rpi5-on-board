import numpy as np
import math
from .config import *

class OccupancyGrid:
    def __init__(self):
        self.size = MAP_SIZE_CELLS
        self.grid = np.zeros((self.size, self.size), dtype=np.int8)
        self.origin = self.size // 2

    def world_to_map(self, x, y):
        mx = int(x / MAP_RESOLUTION) + self.origin
        my = int(y / MAP_RESOLUTION) + self.origin
        return mx, my

    def update_with_tof(self, x, y, theta, tof_readings):
        for dist_mm, angle in zip(tof_readings, TOF_ANGLES):
            if dist_mm <= 0:
                continue

            dist = dist_mm / 1000.0
            wx = x + dist * math.cos(theta + angle)
            wy = y + dist * math.sin(theta + angle)

            mx, my = self.world_to_map(wx, wy)
            if 0 <= mx < self.size and 0 <= my < self.size:
                self.grid[my, mx] = min(self.grid[my, mx] + 1, 100)
