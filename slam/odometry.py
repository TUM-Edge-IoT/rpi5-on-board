import math
from .config import *

class Odometry:
    """
    Single-encoder odometry with IMU-based rotation.
    
    ENCODER FORMAT:
    - AS5600 sends angle in degrees (0-360)
    - Increasing values = forward motion
    - Decreasing values = backward motion
    - Handles wraparound: 359° → 1° = +2° forward, not -358°
    
    WHY SINGLE ENCODER?
    - We only have one reliable encoder on the robot.
    - The encoder tells us how far we've traveled (forward/backward).
    - The IMU gyroscope tells us how much we've rotated.
    
    THE TURN DETECTION PROBLEM:
    - When the robot turns in place (spinning left/right), the encoder wheel
      still rotates, but this rotation does NOT represent forward motion.
    - If we blindly use encoder values during turns, we'd incorrectly think
      the robot moved forward, corrupting our position estimate.
    
    THE SOLUTION:
    - Track the command sent to the robot ("left", "right", "forward", etc.)
    - If command is "left" or "right": robot is turning → ignore encoder
    - If command is "forward"/"backward"/"stop": use encoder for distance
    - Always integrate gyro_z for rotation (works in all cases)
    """
    
    def __init__(self):
        # Robot pose in world coordinates
        self.x = 0.0      # meters, positive = right
        self.y = 0.0      # meters, positive = down (image coordinates)  
        self.theta = 0.0  # radians, start facing up (negative Y in image = up on screen)
        
        # Previous encoder reading for delta calculation
        self.last_angle = None
        
    def update(self, angle, gyro_z, dt, is_turning=False):
        """
        Update robot pose based on single encoder and gyroscope.
        
        Args:
            angle: Current encoder angle (0-360 degrees)
            gyro_z: Angular velocity around Z axis (rad/s, positive = counter-clockwise)
            dt: Time delta since last update (seconds)
            is_turning: True if robot is currently executing a turn command (left/right)
            
        Returns:
            Tuple (x, y, theta) - updated robot pose
        """
        
        # =================================================================
        # STEP 1: Initialize baseline on first call
        # =================================================================
        # The encoder sends angle in degrees (0-360). On the first reading,
        # we just store it as our reference point. We can't calculate 
        # movement without knowing where we started.
        if self.last_angle is None:
            self.last_angle = angle
            return self.x, self.y, self.theta
        
        # =================================================================
        # STEP 2: Calculate encoder delta with WRAPAROUND handling
        # =================================================================
        # The encoder outputs 0-360 degrees. When the wheel rotates past
        # 360, it wraps to 0 (and vice versa for backward motion).
        #
        # Examples:
        #   - 10° → 15°  = +5° (forward, no wrap)
        #   - 15° → 10°  = -5° (backward, no wrap)
        #   - 355° → 5°  = +10° (forward, wrapped past 360)
        #   - 5° → 355°  = -10° (backward, wrapped past 0)
        #
        # The key insight: if |delta| > 180, we wrapped around
        
        delta_angle = angle - self.last_angle
        
        # Handle wraparound
        if delta_angle > ENCODER_WRAP_THRESHOLD:
            # Wrapped backward (e.g., 5° → 355° means we went -10°)
            delta_angle -= ENCODER_MAX
        elif delta_angle < -ENCODER_WRAP_THRESHOLD:
            # Wrapped forward (e.g., 355° → 5° means we went +10°)
            delta_angle += ENCODER_MAX
        
        self.last_angle = angle
        
        # =================================================================
        # STEP 3: ALWAYS update rotation from gyroscope
        # =================================================================
        # The IMU gyroscope measures angular velocity (how fast we're rotating).
        # By integrating over time (multiplying by dt), we get angle change.
        # 
        # This works whether the robot is:
        #   - Moving straight (small gyro_z from drift)
        #   - Turning in place (large gyro_z)
        #   - Moving in an arc (medium gyro_z)
        #
        # Formula: Δθ = ω × Δt  (angular velocity × time = angle change)
        dtheta = gyro_z * dt
        self.theta += dtheta
        
        # =================================================================
        # STEP 4: Update position (only when NOT turning)
        # =================================================================
        # is_turning is set to True when the robot receives "left" or "right"
        # commands. During these commands, the encoder values don't represent
        # forward/backward motion, so we skip position updates.
        #
        # When is_turning is False (forward/backward/stop commands), we use
        # the encoder to calculate how far we've moved.
        
        if not is_turning:
            # Convert encoder degrees to meters
            # The encoder outputs 0-360° per wheel revolution
            # Formula: distance = (delta_degrees / 360) × wheel_circumference
            #        = (delta_degrees / 360) × (2πr)
            meters_per_degree = (2 * math.pi * WHEEL_RADIUS) / ENCODER_MAX
            ds = delta_angle * meters_per_degree  # distance traveled
            
            # Update position using current heading (theta)
            # x component: how far right we moved = distance × cos(heading)
            # y component: how far up we moved = distance × sin(heading)
            #
            # Example: if theta = π/2 (facing up):
            #   cos(π/2) = 0 → no x movement
            #   sin(π/2) = 1 → all movement goes to y
            self.x += ds * math.cos(self.theta)
            self.y += ds * math.sin(self.theta)
            
        # If is_turning == True, we skip position update entirely.
        # The robot rotated but didn't translate.
        
        return self.x, self.y, self.theta