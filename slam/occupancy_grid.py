import numpy as np
import math
from .config import MAP_SIZE_CELLS, MAP_RESOLUTION, TOF_ANGLES, TOF_MAX_RANGE, TOF_MOUNT_OFFSET

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
        
        The ToF sensors are mounted on a semicircular bracket 35mm in front of
        the robot's center. We account for this offset when calculating obstacle
        positions.
        """

        # The ESP32 sends a dict {"d1": 100, "d2": 200...}
        # We must convert this to a sorted list of integers [100, 200...]
        # to match the order of TOF_ANGLES.
        if isinstance(tof_readings, dict):
            # Sort keys (d1, d2, d3, d4) to ensure correct angle mapping
            sorted_keys = sorted(tof_readings.keys())
            readings_list = []
            for k in sorted_keys:
                try:
                    readings_list.append(int(tof_readings[k]))
                except (ValueError, TypeError):
                    readings_list.append(0) # Default to 0 if garbage data
        else:
            # If it's already a list, ensure elements are ints
            try:
                readings_list = [int(r) for r in tof_readings]
            except (ValueError, TypeError):
                readings_list = [0] * len(TOF_ANGLES)


        # Process each ToF sensor reading
        for dist_mm, sensor_angle in zip(readings_list, TOF_ANGLES):
            
            # =================================================================
            # Calculate sensor position (accounting for mount offset)
            # =================================================================
            # The sensor is mounted 35mm in front of the robot center,
            # at a specific angle on the semicircular bracket.
            # 
            # Sensor position in world coordinates:
            #   sensor_x = robot_x + offset * cos(robot_heading)
            #   sensor_y = robot_y + offset * sin(robot_heading)
            #
            # For simplicity, we approximate by adding the offset along the
            # sensor's ray direction (good enough for 35mm offset vs 2m range)
            
            # World angle of this sensor's ray
            world_angle = theta + sensor_angle
            
            # Determine if we hit an obstacle or reached max range
            if dist_mm > 0 and dist_mm < TOF_MAX_RANGE * 2000:
                # Obstacle detected at this distance
                # Add mount offset since sensor is 35mm in front of robot center
                dist = (dist_mm / 1000.0) + TOF_MOUNT_OFFSET
                hit_obstacle = True
            else:
                # No obstacle - use max range (mark as free space)
                dist = TOF_MAX_RANGE + TOF_MOUNT_OFFSET
                hit_obstacle = False

            # Robot position in map coordinates (ray starts from robot center)
            rx, ry = self.world_to_map(x, y)

            # Endpoint in world coordinates
            wx = x + dist * math.cos(world_angle)
            wy = y + dist * math.sin(world_angle)

            # Endpoint in map coordinates
            mx, my = self.world_to_map(wx, wy)

            # Trace ray from robot to endpoint using Bresenham
            ray_cells = self.bresenham_line(rx, ry, mx, my)

            # Mark all cells along the ray as FREE
            # Use stronger decrement to balance with occupied increment
            for cx, cy in ray_cells[:-1] if hit_obstacle else ray_cells:
                if 0 <= cx < self.size and 0 <= cy < self.size:
                    # Decrement towards free (min at -100)
                    # Use -3 to balance: a few free observations can clear a wall
                    self.grid[cy, cx] = max(self.grid[cy, cx] - 3, -100)

            # Mark the endpoint as OCCUPIED only if we hit an obstacle
            if hit_obstacle and 0 <= mx < self.size and 0 <= my < self.size:
                # Increment towards occupied (max at 100)
                # Use +5 for walls - requires ~2 free observations to clear
                self.grid[my, mx] = min(self.grid[my, mx] + 5, 100)
