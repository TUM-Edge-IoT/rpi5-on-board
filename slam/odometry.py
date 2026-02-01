import math
from .config import *

class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Add variables to store the previous tick counts
        self.last_ticks_l = None
        self.last_ticks_r = None

    def update(self, ticks_l, ticks_r, gyro_z, dt):
        # 1. If this is the first time receiving data, treat it as our "0" (baseline)
        if self.last_ticks_l is None:
            self.last_ticks_l = ticks_l
            self.last_ticks_r = ticks_r
            return self.x, self.y, self.theta

        # 2. Calculate the CHANGE (delta) in ticks
        delta_ticks_l = ticks_l - self.last_ticks_l
        delta_ticks_r = ticks_r - self.last_ticks_r

        # 3. Update the stored values for the next loop
        self.last_ticks_l = ticks_l
        self.last_ticks_r = ticks_r

        # 4. Calculate physical distance using the DELTAS, not the raw values
        meters_per_tick = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV

        dl = delta_ticks_l * meters_per_tick
        dr = delta_ticks_r * meters_per_tick

        ds = (dl + dr) / 2.0
        dtheta_enc = (dr - dl) / WHEEL_BASE
        dtheta_imu = gyro_z * dt

        # Complementary filter
        dtheta = 0.98 * dtheta_imu + 0.02 * dtheta_enc

        self.theta += dtheta
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)

        return self.x, self.y, self.theta