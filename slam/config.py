import math

# Robot geometry
WHEEL_BASE = 0.165  # meters
TICKS_PER_REV = 4096
WHEEL_RADIUS = 0.032  # meters

# ToF angles (radians)
TOF_ANGLES = [
    math.radians(-37.5),
    math.radians(-12.5),
    math.radians(12.5),
    math.radians(37.5),
]

# Map
MAP_RESOLUTION = 0.05  # meters per cell
MAP_SIZE_METERS = 10.0
MAP_SIZE_CELLS = int(MAP_SIZE_METERS / MAP_RESOLUTION)
