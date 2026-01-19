import numpy as np
import math
from .config import MAP_SIZE_CELLS, MAP_RESOLUTION, TOF_ANGLES, TOF_MAX_RANGE

# Cell values
CELL_UNKNOWN = 0
CELL_FREE = -10      # Negative = free (decrement towards free)
CELL_OCCUPIED = 100  # Positive = occupied (increment towards wall)

class OccupancyGrid:
    def __init__(self):
        self.size = MAP_SIZE_CELLS
        # Grid values: 0 = unknown, negative = free, positive = occupied
        self.grid = np.zeros((self.size, self.size), dtype=np.int8)
        self.origin = self.size // 2

    def world_to_map(self, x, y):
        mx = int(x / MAP_RESOLUTION) + self.origin
        my = int(y / MAP_RESOLUTION) + self.origin
        return mx, my

    def bresenham_line(self, x0, y0, x1, y1):
        """
        Bresenham's line algorithm - returns all cells along a line
        """
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return cells

    def update_with_tof(self, x, y, theta, tof_readings):
        """
        Update occupancy grid with ToF readings.
        Traces rays from robot to obstacle, marking FREE cells along the path
        and OCCUPIED cell at the endpoint (if obstacle detected).
        If no obstacle detected (reading is 0), marks ray to max range as FREE.
        """
        # Robot position in map coordinates
        rx, ry = self.world_to_map(x, y)
        
        for dist_mm, angle in zip(tof_readings, TOF_ANGLES):
            # Determine if we hit an obstacle or reached max range
            if dist_mm > 0:
                # Obstacle detected at this distance
                dist = dist_mm / 1000.0  # Convert to meters
                hit_obstacle = True
            else:
                # No obstacle - use max range (mark as free space)
                dist = TOF_MAX_RANGE
                hit_obstacle = False
            
            # Endpoint in world coordinates
            wx = x + dist * math.cos(theta + angle)
            wy = y + dist * math.sin(theta + angle)
            
            # Endpoint in map coordinates
            mx, my = self.world_to_map(wx, wy)
            
            # Trace ray from robot to endpoint using Bresenham
            ray_cells = self.bresenham_line(rx, ry, mx, my)
            
            # Mark all cells along the ray as FREE
            for cx, cy in ray_cells[:-1] if hit_obstacle else ray_cells:
                if 0 <= cx < self.size and 0 <= cy < self.size:
                    # Decrement towards free (min at -100)
                    self.grid[cy, cx] = max(self.grid[cy, cx] - 1, -100)
            
            # Mark the endpoint as OCCUPIED only if we hit an obstacle
            if hit_obstacle and 0 <= mx < self.size and 0 <= my < self.size:
                # Increment towards occupied (max at 100)
                self.grid[my, mx] = min(self.grid[my, mx] + 10, 100)
