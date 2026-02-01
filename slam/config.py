import math

# Robot geometry
WHEEL_BASE = 0.160
TICKS_PER_REV = 4096
WHEEL_RADIUS = 0.06551

# --- NEW: Accelerometer Physics Constants ---
# Ignore acceleration below this value (m/s^2) to prevent drift while standing still
# Earth gravity is ~9.8. Noise is usually 0.1 to 0.5.
ACCEL_THRESHOLD = 0.2 

# "Friction" factor (0.0 to 1.0). 
# If accel is 0, multiply velocity by this to slow down the map marker.
# 0.9 means it loses 10% speed every update loop.
VELOCITY_DECAY = 0.8 
# --------------------------------------------

# ToF angles
TOF_ANGLES = [
    math.radians(-37.5),
    math.radians(-12.5),
    math.radians(12.5),
    math.radians(37.5),
]

TOF_MAX_RANGE = 2.0 # Changed to meters (2.0) based on typical usage
MAP_RESOLUTION = 0.05
MAP_SIZE_METERS = 20.0
MAP_SIZE_CELLS = int(MAP_SIZE_METERS / MAP_RESOLUTION)