import math
from .config import *

class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, ticks_l, ticks_r, gyro_z, dt):
        meters_per_tick = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV

        dl = ticks_l * meters_per_tick
        dr = ticks_r * meters_per_tick

        ds = (dl + dr) / 2.0
        dtheta_enc = (dr - dl) / WHEEL_BASE
        dtheta_imu = gyro_z * dt

        # Complementary filter
        dtheta = 0.98 * dtheta_imu + 0.02 * dtheta_enc

        self.theta += dtheta
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)

        return self.x, self.y, self.theta
