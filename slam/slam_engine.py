import time
from .odometry import Odometry
from .occupancy_grid import OccupancyGrid

class SlamEngine:
    def __init__(self):
        self.odom = Odometry()
        self.map = OccupancyGrid()
        self.last_ts = None

    def process(self, packet):
        ts = packet["ts"] / 1000.0
        if self.last_ts is None:
            self.last_ts = ts
            return None

        dt = ts - self.last_ts
        self.last_ts = ts

        x, y, theta = self.odom.update(
            packet["enc"]["l"],
            packet["enc"]["r"],
            packet["imu"]["gz"],
            dt
        )

        self.map.update_with_tof(x, y, theta, packet["tof"])

        return {
            "pose": {"x": x, "y": y, "theta": theta},
            "map": self.map.grid
        }
