import math
from .config import *

class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, gyro_z, dt, cmd):
        # --- 1. Rotation (Use Real Sensor Data) ---
        # The gyro gives us the true turning speed, regardless of commands.
        # This handles bumps, slips, or manual pushing correctly.
        dtheta = gyro_z * dt
        
        # --- 2. Linear Distance (Dead Reckoning) ---
        # We assume if the command is 'F' or 'B', the motors are spinning
        # at the calibrated speed.
        linear_vel = 0.0
        
        if cmd == 'F':
            linear_vel = ROBOT_SPEED_MPS
        elif cmd == 'B':
            linear_vel = -ROBOT_SPEED_MPS
        # Note: If cmd is 'L', 'R', or 'S', linear_vel is 0. 
        # This assumes the robot turns in place (tank turn).
        
        ds = linear_vel * dt

        # --- 3. Update Position ---
        self.theta += dtheta
        
        # Normalize theta to -pi to pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)

        return self.x, self.y, self.theta