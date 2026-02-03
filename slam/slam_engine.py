import time
from .odometry import Odometry
from .occupancy_grid import OccupancyGrid

class SlamEngine:
    def __init__(self):
        self.odom = Odometry()
        self.map = OccupancyGrid()
        self.last_ts = None
        self.is_turning = False  # Track if robot is currently turning

    def set_turning(self, turning):
        """
        Set the turning state. Called when robot receives left/right commands.
        
        Args:
            turning: True if robot is turning (left/right command),
                    False if moving straight or stopped
        """
        self.is_turning = turning

    def process(self, packet):
        ts = packet["ts"] / 1000.0
        if self.last_ts is None:
            self.last_ts = ts
            return None

        dt = ts - self.last_ts
        self.last_ts = ts

        # Single encoder odometry: use only one encoder value
        # We use "enc" which is set in uart2mqtt.py from the ESP32's encoder data
        encoder_ticks = packet["enc"]
        
        # Pass is_turning flag to odometry so it knows when to ignore encoder
        x, y, theta = self.odom.update(
            encoder_ticks,
            packet["imu"]["gz"],
            dt,
            is_turning=self.is_turning
        )

        self.map.update_with_tof(x, y, theta, packet["tof"])

        return {
            "pose": {"x": x, "y": y, "theta": theta},
            "map": self.map.grid
        }
