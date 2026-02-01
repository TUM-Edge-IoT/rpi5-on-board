import math
from .config import *

class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Physics State
        self.vx = 0.0 # Velocity X (World Frame)
        self.vy = 0.0 # Velocity Y (World Frame)

    def update(self, gyro_z, accel_x, dt):
        """
        Calculate position using Double Integration of Acceleration.
        accel_x: Forward acceleration in m/s^2
        gyro_z:  Rotation speed in rad/s
        dt:      Time since last update in seconds
        """
        
        # 1. Update Heading (Gyro is reliable)
        self.theta += gyro_z * dt
        # Normalize theta (-pi to pi)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 2. Filter Accelerometer Noise (The "Dead Zone")
        # If acceleration is tiny, treat it as 0 to stop drift
        if abs(accel_x) < ACCEL_THRESHOLD:
            ax = 0.0
        else:
            ax = accel_x

        # 3. Calculate World Acceleration Vectors
        # We assume the robot only accelerates 'forward' relative to itself
        # Ax_world = ax_robot * cos(theta)
        # Ay_world = ax_robot * sin(theta)
        world_ax = ax * math.cos(self.theta)
        world_ay = ax * math.sin(self.theta)

        # 4. Update Velocity (Integration 1: a -> v)
        self.vx += world_ax * dt
        self.vy += world_ay * dt

        # 5. Apply "Virtual Friction"
        # If the robot is not accelerating, we must dampen velocity 
        # otherwise it will glide on the map forever like it's on ice.
        if ax == 0.0:
            self.vx *= VELOCITY_DECAY
            self.vy *= VELOCITY_DECAY
            
            # Snap to 0 if very slow
            if abs(self.vx) < 0.01: self.vx = 0
            if abs(self.vy) < 0.01: self.vy = 0

        # 6. Update Position (Integration 2: v -> x)
        self.x += self.vx * dt
        self.y += self.vy * dt

        return self.x, self.y, self.theta