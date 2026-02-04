# SLAM System Documentation

## Overview

This document explains the **Simultaneous Localization and Mapping (SLAM)** system implemented for the RPi5 robotics platform. The system runs on a Raspberry Pi 5 and processes sensor data from an ESP32 microcontroller to build a 2D occupancy grid map while tracking the robot's position.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Design Decisions](#design-decisions)
3. [Algorithms in Detail](#algorithms-in-detail)
4. [Module Breakdown](#module-breakdown)
5. [Robot Visualization](#robot-visualization)
6. [MQTT Protocol](#mqtt-protocol)
7. [Tuning Guide](#tuning-guide)
8. [Limitations & Future Work](#limitations--future-work)

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ESP32 (Firmware)                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐                    │
│  │ Encoder  │  │   IMU    │  │   ToF    │  │  Motor   │                    │
│  │ AS5600   │  │ MPU6050  │  │ VL53L0X  │  │ Control  │                    │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────▲─────┘                    │
│       │             │             │             │                          │
│       └─────────────┴─────────────┘             │                          │
│                     │                           │                          │
│              JSON over UART ──────────► Command parsing                    │
└─────────────────────┼───────────────────────────┼──────────────────────────┘
                      │                           │
                      ▼                           │
┌─────────────────────────────────────────────────┼──────────────────────────┐
│                       Raspberry Pi 5                                       │
│  ┌──────────────────────────────────────────────┼─────────────────────────┐│
│  │                    uart2mqtt.py              │                         ││
│  │  ┌─────────────┐   ┌─────────────┐   ┌───────┴───────┐                 ││
│  │  │ Serial Read │──►│ JSON Parse  │──►│ Command Track │◄── MQTT Sub    ││
│  │  └─────────────┘   └──────┬──────┘   │ (left/right)  │                 ││
│  │                           │          └───────────────┘                 ││
│  │                           ▼                                            ││
│  │  ┌────────────────────────────────────────────────────────────────────┐││
│  │  │                      SLAM Engine                                   │││
│  │  │  ┌─────────────┐      ┌─────────────────┐      ┌────────────────┐ │││
│  │  │  │  Odometry   │─────►│  Occupancy Grid │─────►│  Map Publish   │ │││
│  │  │  │ (position)  │      │   (obstacles)   │      │  (MQTT/base64) │ │││
│  │  │  └─────────────┘      └─────────────────┘      └────────────────┘ │││
│  │  └────────────────────────────────────────────────────────────────────┘││
│  └────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────┘
                      │
                      ▼ MQTT
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Cloud Backend                                     │
│        Receives: robots/{id}/slam/pose, robots/{id}/slam/map               │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Design Decisions

### 1. Map Resolution: 5cm per Cell

**Decision:** Each cell in our occupancy grid represents a 5cm × 5cm physical area.

**Why 5cm?**

| Resolution | Grid Size for 20m² | Memory | Pros | Cons |
|------------|-------------------|--------|------|------|
| 1cm | 2000×2000 | 4MB | Very detailed | Too much noise, slow |
| **5cm** | **400×400** | **160KB** | **Good balance** | **Chosen** |
| 10cm | 200×200 | 40KB | Fast, low memory | Misses doorways |
| 20cm | 100×100 | 10KB | Very fast | Unusable for navigation |

**Key considerations:**
- **Robot width:** 153mm ≈ 3 cells → Robot is visible on map
- **Doorway detection:** 80cm door = 16 cells → Clearly visible
- **ToF accuracy:** VL53L0X has ±3% error at 2m = ±6cm → 5cm cells average out noise
- **Transmission:** 160KB compresses to ~2-5KB with zlib → Fits in MQTT easily

---

### 2. ToF Max Range: 3 meters

**Decision:** We cap ToF readings at 3 meters even though VL53L0X can sometimes read further.

**Why 3m?**

```
Distance    │ VL53L0X Accuracy  │ Our Decision
────────────┼───────────────────┼──────────────────────────
0 - 1m      │ ±3% (~3cm)        │ ✓ High confidence
1 - 2m      │ ±5% (~10cm)       │ ✓ Good confidence  
2 - 3m      │ ±10% (~30cm)      │ ✓ Acceptable
3m+         │ Unreliable        │ ✗ Ignore readings
```

**Trade-off:**
- **Longer range** = See further, but noisy readings corrupt map
- **Shorter range** = Less noise, but slower exploration
- **3m** = Good balance for indoor environments (rooms are typically 3-6m)

---

### 3. Single Encoder + Command-Based Turn Detection

**The Problem:**

We only have one working encoder. With two encoders, we could calculate rotation from the difference in wheel speeds. With one encoder, we can't.

**Rejected Alternative: Gyroscope Threshold**

We considered detecting turns by checking if `|gyro_z| > threshold`:

```
❌ Problem: Threshold is arbitrary
   - Too low: Ignores encoder during gentle curves
   - Too high: Uses encoder during slow turns
   - Different surfaces/batteries change turn speed
```

**Chosen Solution: Command-Based Detection**

```
✓ When robot receives "left" or "right" → We KNOW it's turning
✓ No threshold tuning needed
✓ 100% accurate detection
✓ Works at any turn speed
```

**Why always use the gyroscope (even during straight motion)?**

| Sensor | What it measures | When useful |
|--------|------------------|-------------|
| Encoder | Distance traveled | Forward/backward motion |
| Gyroscope | Rotation rate | **ALWAYS** (even small drifts) |

The gyroscope catches:
- Uneven ground causing drift
- Hitting obstacles and deflecting
- Wheel slip on smooth surfaces

Without gyroscope during straight motion, these small rotations accumulate and the map drifts.

---

### 4. Probabilistic Occupancy Grid

**Decision:** Cells aren't binary (free/occupied). They have confidence values from -100 to +100.

**Why probabilistic?**

Real sensors have noise. A single reading might be wrong:
- ToF reflects off shiny surface → false "far" reading
- ToF hits dust particle → false "near" reading

**The Solution: Accumulate Evidence**

```
Cell Value   │ Meaning           │ Confidence
─────────────┼───────────────────┼────────────
-100         │ Definitely FREE   │ 100% certain
 -50         │ Probably free     │ 50% certain
   0         │ UNKNOWN           │ No data
 +50         │ Probably occupied │ 50% certain
+100         │ Definitely WALL   │ 100% certain
```

**Update Rules:**

```python
# Seeing FREE space: decrement slowly
cell = max(cell - 1, -100)

# Seeing OBSTACLE: increment quickly  
cell = min(cell + 10, +100)
```

**Why asymmetric (10 vs 1)?**

| Observation | Confidence | Reason |
|-------------|------------|--------|
| "I see an obstacle" | HIGH (+10) | ToF hit something real |
| "I see empty space" | LOW (-1) | Maybe sensor didn't reach that far |

This means:
- **Obstacles appear quickly** (1-2 readings)
- **Obstacles disappear slowly** (10+ readings)
- **Prevents flickering** when sensor is noisy

**Self-Correction Example:**

```
Time 0: Cell is FREE (-50)
Time 1: ToF incorrectly reads obstacle → -50 + 10 = -40 (still free)
Time 2: ToF correctly reads free → -40 - 1 = -41 (more free)
Time 3: ToF correctly reads free → -41 - 1 = -42 (confirmed free)
...
The false reading was absorbed by the probabilistic model!
```

---

## Algorithms in Detail

### 1. Bresenham's Line Algorithm

**Purpose:** Efficiently find all grid cells that a laser/ToF ray passes through.

**The Problem:**

```
Given: Robot at cell (5, 5), obstacle detected at cell (12, 8)
Find: All cells between them that the ray passes through
```

**Why not just use floating-point math?**

```python
# Naive approach (SLOW)
for t in range(0, 100):
    x = 5 + t * (12-5) / 100
    y = 5 + t * (8-5) / 100
    cells.add((int(x), int(y)))
```

Problems:
- Floating-point operations are slow
- `t` step size affects accuracy
- May miss cells or duplicate cells

**Bresenham's Solution: Integer-Only Math**

The algorithm uses only addition, subtraction, and bit shifts:

```python
def bresenham_line(x0, y0, x1, y1):
    """
    Returns all cells along a line from (x0,y0) to (x1,y1).
    Uses only integer arithmetic for speed.
    """
    cells = []
    dx = abs(x1 - x0)  # Horizontal distance
    dy = abs(y1 - y0)  # Vertical distance
    
    # Direction of movement
    sx = 1 if x0 < x1 else -1  # Step in x: +1 or -1
    sy = 1 if y0 < y1 else -1  # Step in y: +1 or -1
    
    # Error accumulator (the key insight!)
    err = dx - dy
    
    while True:
        cells.append((x0, y0))
        
        if x0 == x1 and y0 == y1:
            break
            
        e2 = 2 * err
        
        # Should we step in X?
        if e2 > -dy:
            err -= dy
            x0 += sx
            
        # Should we step in Y?
        if e2 < dx:
            err += dx
            y0 += sy
            
    return cells
```

**How it works (intuition):**

Imagine walking from start to end. At each step, you must decide: go horizontal or vertical?

```
         End ●
            /
           /    ← The "ideal" line
          /
         ╱
        ╱
Start ●

The error tracks: "How far am I from the ideal line?"
- If error > 0: I'm above the line, step DOWN (or right)
- If error < 0: I'm below the line, step UP (or up)
```

**Visual Example:**

```
Ray from (0,0) to (7,3):

  3 │        ●━━━● End
    │      ●━┛
  2 │    ●━┛
    │  ●━┛
  1 │●━┛
    │
  0 ●─────────────
    0 1 2 3 4 5 6 7
    
Cells returned: (0,0), (1,0), (2,1), (3,1), (4,2), (5,2), (6,3), (7,3)
```

**Performance:**
- **O(max(dx, dy))** - Linear in line length
- **No floating-point** - Fast on embedded systems
- **No missed cells** - Mathematically complete

---

### 2. ToF Ray Integration

**Purpose:** Convert 4 ToF distance readings into map updates.

**Step-by-Step Process:**

```
┌─────────────────────────────────────────────────────────────────────┐
│ INPUT: Robot pose (x, y, θ) and ToF readings {d1, d2, d3, d4}      │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 1: Parse ToF readings                                          │
│                                                                     │
│   ESP32 sends: {"d1": 1500, "d2": 1200, "d3": 1100, "d4": 1800}    │
│                                                                     │
│   We sort keys alphabetically: d1, d2, d3, d4                       │
│   This ensures correct mapping to angles!                           │
│                                                                     │
│   readings = [1500, 1200, 1100, 1800]  (millimeters)               │
│   angles   = [-49.5°, -16.5°, +16.5°, +49.5°]                      │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 2: For each sensor, calculate ray endpoint                    │
│                                                                     │
│   world_angle = robot_theta + sensor_angle                          │
│                                                                     │
│   Example: Robot facing up (θ = 90°), sensor d1 at -49.5°          │
│            world_angle = 90° + (-49.5°) = 40.5° from +X axis       │
│                                                                     │
│   distance = reading_mm / 1000 + MOUNT_OFFSET                       │
│            = 1500 / 1000 + 0.035                                    │
│            = 1.535 meters                                           │
│                                                                     │
│   endpoint_x = robot_x + distance × cos(world_angle)                │
│   endpoint_y = robot_y + distance × sin(world_angle)                │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 3: Convert to grid coordinates                                 │
│                                                                     │
│   Grid origin is at center (cell 200, 200)                          │
│                                                                     │
│   grid_x = (world_x / 0.05) + 200                                   │
│   grid_y = (world_y / 0.05) + 200                                   │
│                                                                     │
│   Example: world position (1.5, 2.0) meters                         │
│            grid_x = (1.5 / 0.05) + 200 = 30 + 200 = 230            │
│            grid_y = (2.0 / 0.05) + 200 = 40 + 200 = 240            │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 4: Trace ray using Bresenham's algorithm                      │
│                                                                     │
│   ray_cells = bresenham(robot_grid, endpoint_grid)                  │
│                                                                     │
│   Example: Robot at (200, 200), endpoint at (230, 215)              │
│            Returns ~35 cells along the ray                          │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 5: Update grid cells                                          │
│                                                                     │
│   If ToF reading > 0 (obstacle detected):                           │
│       ├── All cells EXCEPT last: mark as FREE (decrement by 1)     │
│       └── Last cell: mark as OCCUPIED (increment by 10)            │
│                                                                     │
│   If ToF reading = 0 (no obstacle / max range):                     │
│       └── ALL cells: mark as FREE (decrement by 1)                 │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ Robot ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━● Obstacle        │   │
│   │       │ -1  -1  -1  -1  -1  -1  -1  -1  │ +10               │   │
│   │       │        All these: FREE          │ This: OCCUPIED    │   │
│   └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

**Why add MOUNT_OFFSET (35mm)?**

The ToF sensors are mounted on a bracket 35mm in front of the robot's center point. The ToF measures distance from the sensor, not from the robot center.

```
         ToF sensors here
              │
              ▼
        ┌─────────────┐
        │  ●  ●  ●  ● │ ← Bracket (35mm from center)
        │             │
        │      ●      │ ← Robot center (odometry reference)
        │             │
        └─────────────┘
              │
              ▼
         This is where (x, y) refers to

So: true_distance_from_center = ToF_reading + 35mm
```

---

### 3. Single-Encoder Odometry

**Purpose:** Track robot position using one encoder and gyroscope.

**The Mathematical Model:**

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ODOMETRY UPDATE CYCLE                            │
└─────────────────────────────────────────────────────────────────────┘

INPUT: 
  - ticks: Current encoder reading (absolute count)
  - gyro_z: Angular velocity (rad/s)
  - dt: Time since last update (seconds)
  - is_turning: True if robot is executing left/right command

┌─────────────────────────────────────────────────────────────────────┐
│ STEP 1: Calculate angle delta with WRAPAROUND handling             │
│                                                                     │
│   The AS5600 encoder outputs angle in degrees (0-360).              │
│   When the wheel rotates past 360°, it wraps to 0°.                 │
│                                                                     │
│   delta = current_angle - last_angle                                │
│                                                                     │
│   if delta > 180:   delta -= 360  (wrapped backward)                │
│   if delta < -180:  delta += 360  (wrapped forward)                 │
│                                                                     │
│   Examples:                                                         │
│     10° → 15°   = +5° (forward, no wrap)                            │
│     355° → 5°  = +10° (forward, wrapped past 360)                   │
│     5° → 355°  = -10° (backward, wrapped past 0)                    │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 2: ALWAYS update rotation (even during turns)                 │
│                                                                     │
│   Δθ = gyro_z × dt                                                  │
│                                                                     │
│   Example: gyro_z = 0.5 rad/s, dt = 0.02s                          │
│            Δθ = 0.5 × 0.02 = 0.01 rad ≈ 0.57°                       │
│                                                                     │
│   θ_new = θ_old + Δθ                                                │
│                                                                     │
│   WHY ALWAYS? The gyroscope is reliable whether moving or turning. │
│   It catches small drifts that would otherwise accumulate.         │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 3: Update position ONLY if NOT turning                        │
│                                                                     │
│   if is_turning:                                                    │
│       # Skip position update!                                       │
│       # Encoder ticks during turn don't mean forward motion        │
│       pass                                                          │
│   else:                                                             │
│       # Convert encoder degrees to meters                           │
│       meters_per_degree = (2π × WHEEL_RADIUS) / 360                 │
│                         = (2π × 0.032755) / 360                     │
│                         = 0.000572 meters/degree                    │
│                                                                     │
│       distance = Δangle × meters_per_degree                         │
│                                                                     │
│       # Update position in direction of current heading            │
│       x_new = x_old + distance × cos(θ)                             │
│       y_new = y_old + distance × sin(θ)                             │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ OUTPUT: Updated pose (x, y, θ)                                      │
└─────────────────────────────────────────────────────────────────────┘
```

**Why ignore encoder during turns?**

```
FORWARD MOTION:                    TURN IN PLACE:
                                   
    ●━━━━━━━━━━━━━━●               ●  ↻  ●
    Start        End               Same position!
                                   
Encoder reads: +1000 ticks         Encoder reads: +500 ticks
Meaning: Moved 10cm forward        Meaning: Wheel spun but robot
                                           didn't translate
```

If we used encoder during turns, we'd think the robot moved forward when it just rotated!

---

## Module Breakdown

### `config.py` - Robot Configuration

```python
# Robot wheel geometry
WHEEL_BASE = 0.160       # Distance between wheels (meters)
WHEEL_RADIUS = 0.032755  # Wheel radius (meters) - diameter is 65.51mm

# Encoder configuration (AS5600 outputs angle in degrees)
ENCODER_MAX = 360           # One full wheel revolution (degrees)
ENCODER_WRAP_THRESHOLD = 180  # If |delta| > 180, it's a wraparound

# ToF sensor mount geometry
TOF_ANGLES = [
    math.radians(-49.5),   # d1: Outer left
    math.radians(-16.5),   # d2: Inner left
    math.radians(16.5),    # d3: Inner right
    math.radians(49.5),    # d4: Outer right
]
TOF_MOUNT_OFFSET = 0.035  # Sensors 35mm in front of center
TOF_MAX_RANGE = 3.0       # Maximum reliable range (meters)

# Map configuration
MAP_RESOLUTION = 0.05     # 5cm per cell
MAP_SIZE_METERS = 20.0    # 20m × 20m map
MAP_SIZE_CELLS = 400      # 400 × 400 cells
```

---

### `odometry.py` - Position Tracking

Implements single-encoder odometry with IMU fusion.

**Key method:** `update(ticks, gyro_z, dt, is_turning)`

See [Single-Encoder Odometry](#3-single-encoder-odometry) for algorithm details.

---

### `occupancy_grid.py` - Map Building

Implements probabilistic occupancy grid with Bresenham ray tracing.

**Key method:** `update_with_tof(x, y, theta, tof_readings)`

See [Bresenham's Algorithm](#1-bresenhams-line-algorithm) and [ToF Ray Integration](#2-tof-ray-integration) for algorithm details.

---

### `slam_engine.py` - Orchestrator

Combines odometry and mapping:

```python
class SlamEngine:
    def __init__(self):
        self.odom = Odometry()        # Position tracker
        self.map = OccupancyGrid()    # Map builder
        self.is_turning = False       # Turn detection flag
    
    def set_turning(self, turning):
        """Called when left/right command received"""
        self.is_turning = turning
    
    def process(self, packet):
        # 1. Calculate time delta
        dt = current_ts - last_ts
        
        # 2. Update position (encoder + gyro)
        x, y, theta = self.odom.update(
            encoder_ticks, 
            gyro_z, 
            dt, 
            is_turning=self.is_turning
        )
        
        # 3. Update map with ToF readings
        self.map.update_with_tof(x, y, theta, tof_readings)
        
        # 4. Return pose and map
        return {"pose": {"x": x, "y": y, "theta": theta}, "map": grid}
```

---

### `uart2mqtt.py` - Bridge & Communication

**Responsibilities:**
1. Read UART from ESP32 (sensor data)
2. Parse JSON packets
3. Track commands (detect turns for SLAM)
4. Run SLAM engine
5. Publish to MQTT

**Command Interception for Turn Detection:**

```python
def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    cmd = data["command"]
    
    # Tell SLAM if we're turning
    if cmd in ("left", "right"):
        slam.set_turning(True)   # Ignore encoder
    else:
        slam.set_turning(False)  # Use encoder
    
    # Forward command to ESP32
    ser.write((payload + "\n").encode())
```

---

## Robot Visualization

### Physical Dimensions

```
┌──────────────────────────────────────────────────────────────────┐
│                      ROBOT DIMENSIONS                            │
│                                                                  │
│                         153mm (width)                            │
│                    ◄─────────────────────►                       │
│                                                                  │
│                    ┌─────────────────────┐  ▲                    │
│                    │                     │  │                    │
│                    │    ┌───────────┐    │  │                    │
│                    │    │ ToF Array │    │  │                    │
│                    │    └───────────┘    │  │                    │
│                    │         ▲           │  │                    │
│                    │       Front         │  │ 320mm              │
│                    │                     │  │ (length)           │
│                    │    ┌───────────┐    │  │                    │
│                    │    │   RPi 5   │    │  │                    │
│                    │    └───────────┘    │  │                    │
│                    │                     │  │                    │
│                    └─────────────────────┘  ▼                    │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

### Map Representation

At 5cm/cell resolution:
- **Width:** 153mm ≈ 3 cells
- **Length:** 320mm ≈ 6-7 cells

The robot is drawn as an oriented rectangle that rotates with the robot's heading:

```python
# Robot dimensions in cells
ROBOT_HALF_WIDTH = 2   # ±2 cells = ~15-20cm total width
ROBOT_HALF_LENGTH = 3  # ±3 cells = ~30cm total length

# Draw robot rotated by current heading (theta)
for dy in range(-ROBOT_HALF_LENGTH, ROBOT_HALF_LENGTH + 1):
    for dx in range(-ROBOT_HALF_WIDTH, ROBOT_HALF_WIDTH + 1):
        # Rotate offset by robot heading
        rx = mx + dx * cos(theta) - dy * sin(theta)
        ry = my + dx * sin(theta) + dy * cos(theta)
        display[ry, rx] = 200  # Robot marker
```

**Visual on map:**

```
θ = 90° (facing up):          θ = 45° (facing diagonal):
                              
    ■ ■ ■                           ■
    ■ ■ ■                         ■ ■ ■
    ■ ■ ■                       ■ ■ ■ ■ ■
    ■ ■ ■                         ■ ■ ■
    ■ ■ ■                           ■
    ■ ■ ■
    ■ ■ ■
```

---

## MQTT Protocol

### Topics

| Topic | Direction | Payload |
|-------|-----------|---------|
| `robots/{id}/command` | Cloud → Robot | `{"command": "forward"}` |
| `robots/{id}/slam/pose` | Robot → Cloud | `{"x": 1.5, "y": 2.3, "theta": 1.57}` |
| `robots/{id}/slam/map` | Robot → Cloud | `{"resolution": 0.05, "size": [400,400], "data": "base64..."}` |
| `robots/{id}/status` | Robot → Cloud | `{"status": "online", "ip_address": "..."}` |

### Map Compression

The 400×400 grid would be 160KB uncompressed. We compress it:

```python
# 1. Convert to display values
display = np.zeros_like(raw_map, dtype=np.uint8)
display[raw_map < 0] = 1      # Free space
display[raw_map > 0] = 100    # Occupied
display[robot_cells] = 200    # Robot position

# 2. Compress with zlib
compressed = zlib.compress(display.tobytes())

# 3. Base64 encode for JSON transport
encoded = base64.b64encode(compressed).decode()
```

**Compression results:** 160KB → ~2-5KB (97% reduction)

---

## Tuning Guide

### Problem: Robot drifts sideways during straight motion
- **Cause:** Gyroscope bias or wheel radius error
- **Fix:** Calibrate gyroscope zero offset, measure wheel radius precisely

### Problem: Map is rotated incorrectly
- **Cause:** ToF angles don't match physical sensor positions
- **Fix:** Verify d1=left, d4=right, and angles match mounting

### Problem: Obstacles appear in wrong positions
- **Cause:** Mount offset incorrect or encoder direction inverted
- **Fix:** Measure ToF bracket distance from robot center, check encoder polarity

### Problem: Robot "jumps" during turns
- **Cause:** `set_turning(True)` not called when turning
- **Fix:** Verify command parsing in uart2mqtt.py

### Problem: Map has random noise/flickering
- **Cause:** ToF readings are unstable
- **Fix:** Increase the "free" threshold (require more readings to clear cells)

---

## Limitations & Future Work

### Current Limitations

1. **Single encoder** - Cannot detect wheel slip or skid
2. **No loop closure** - Map drift accumulates over time
3. **100° FOV** - Blind spots on sides and rear
4. **2D only** - No vertical obstacle detection
5. **No relocalization** - If lost, map becomes corrupted

### Future Improvements

- [ ] Add second encoder for differential odometry
- [ ] Implement scan matching (ICP) for loop closure
- [ ] Add ultrasonic sensors for side coverage
- [ ] IMU accelerometer fusion for velocity estimation
- [ ] Save/load maps for persistent navigation
- [ ] Add particle filter for uncertainty estimation

---

## File Structure

```
rpi5-on-board/slam/
├── __init__.py              # Package marker
├── config.py                # Robot/sensor parameters
├── odometry.py              # Position tracking (encoder + IMU)
├── occupancy_grid.py        # Map building (ToF rays + Bresenham)
├── slam_engine.py           # Orchestrator
└── SLAM_DOCUMENTATION.md    # This file

rpi5-on-board/
├── uart2mqtt.py             # Main bridge script
├── Dockerfile               # Container definition
└── requirements.txt         # Python dependencies
```

---

## Quick Reference

### Sensor Data Packet (ESP32 → RPi5)

```json
{
  "timestamp_ms": 123456789,
  "encoders": {"e1": 4096, "e2": 0},
  "imu": {"ax": 0.01, "ay": -0.02, "az": 9.81, "gx": 0.001, "gy": 0.002, "gz": 0.15},
  "tof": {"d1": 1500, "d2": 1200, "d3": 1100, "d4": 1800}
}
```

### Key Constants

| Constant | Value | Unit | Description |
|----------|-------|------|-------------|
| `WHEEL_RADIUS` | 0.032755 | m | Wheel radius (diameter 65.51mm) |
| `ENCODER_MAX` | 360 | degrees | One full wheel revolution |
| `ENCODER_WRAP_THRESHOLD` | 180 | degrees | Wraparound detection threshold |
| `MAP_RESOLUTION` | 0.05 | m/cell | Grid cell size |
| `TOF_MAX_RANGE` | 3.0 | m | Max reliable distance |
| `TOF_MOUNT_OFFSET` | 0.035 | m | Sensor bracket offset |

### Cell Update Rules

| Event | Action | Value Change |
|-------|--------|--------------|
| Ray passes through cell | Mark FREE | -1 (min -100) |
| Ray ends at cell (obstacle) | Mark OCCUPIED | +10 (max +100) |
