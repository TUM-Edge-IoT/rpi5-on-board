import math

# Robot geometry
WHEEL_BASE = 0.165  # meters
TICKS_PER_REV = 4096
WHEEL_RADIUS = 0.032  # meters

# ToF angles (radians) - 4 sensors at 25° intervals centered on front
# Sensors at: -37.5°, -12.5°, +12.5°, +37.5° from front
TOF_ANGLES = [
    math.radians(-37.5),
    math.radians(-12.5),
    math.radians(12.5),
    math.radians(37.5),
]

# ToF max range in meters (VL53L0X typical max ~2m)
TOF_MAX_RANGE = 2.0

# Map
MAP_RESOLUTION = 0.05  # meters per cell
MAP_SIZE_METERS = 20.0  # 20m x 20m map
MAP_SIZE_CELLS = int(MAP_SIZE_METERS / MAP_RESOLUTION)  # 400 cells
