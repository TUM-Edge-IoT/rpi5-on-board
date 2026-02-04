import math

# Robot geometry
WHEEL_BASE = 0.160 # meters
WHEEL_RADIUS = 0.032755  # meters (diameter is 0.06551m)

# Encoder configuration
# AS5600 sends angle in degrees (0-360), not raw ticks
# One full wheel revolution = 360 degrees
ENCODER_MAX = 360  # Maximum encoder value (degrees)
ENCODER_WRAP_THRESHOLD = 180  # If delta > 180, it's a wraparound



# ToF sensor mount geometry
# The sensors are mounted on a semicircular bracket:
#   - Bracket radius: 35mm (sensors are 35mm in front of robot center)
#   - Bracket width: 81.71mm side-to-side
#   - 4 sensors total, each 33° apart from neighbors
#   - Total coverage: ~100° (NOT 180°, sensors don't reach the sides)
#
# Sensor layout (robot-local frame, 0° = front):
#   - Outer left:  -49.5° (16.5° + 33°)
#   - Inner left:  -16.5° (half of 33° gap from center)
#   - Inner right: +16.5°
#   - Outer right: +49.5°
#
# Physical positions on bracket (12.5mm from edge, 30mm between pairs)
# NOTE: In standard math convention, positive angles = counter-clockwise = LEFT
#       So left sensors get POSITIVE angles, right sensors get NEGATIVE angles
# IMPORTANT: Adding 180° (π) offset because the ToF sensors are mounted facing
#            the opposite direction from the robot's "front" in our coordinate system
TOF_ANGLES = [
    math.radians(49.5 + 180),    # d1: Outer left sensor (counter-clockwise from front)
    math.radians(16.5 + 180),    # d2: Inner left sensor
    math.radians(-16.5 + 180),   # d3: Inner right sensor
    math.radians(-49.5 + 180),   # d4: Outer right sensor (clockwise from front)
]

# ToF sensor mount offset from robot center (meters)
# The semicircle bracket places sensors 35mm in front of the robot's center point
TOF_MOUNT_OFFSET = 0.035  # 35mm

# ToF max range in meters (VL53L0X typical max ~2m, we use 3m for safety)
TOF_MAX_RANGE = 3.0  # meters (was incorrectly 3000 = 3km!)

# Map
MAP_RESOLUTION = 0.05  # meters per cell
MAP_SIZE_METERS = 20.0  # 20m x 20m map
MAP_SIZE_CELLS = int(MAP_SIZE_METERS / MAP_RESOLUTION)  # 400 cells
