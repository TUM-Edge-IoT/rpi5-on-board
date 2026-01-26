# RPi5 On-Board System

This folder contains all code that runs on the Raspberry Pi 5, which acts as the bridge between the ESP32 (sensors/motors) and the cloud backend.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Raspberry Pi 5                              │
│                                                                 │
│  ┌──────────────┐    ┌─────────────────────────────────────┐   │
│  │   Camera     │───▶│  mediamtx (RTSP Server)             │   │
│  │  (USB/CSI)   │    │  + FFmpeg streamer                  │───┼──▶ RTSP Stream
│  └──────────────┘    └─────────────────────────────────────┘   │
│                                                                 │
│  ┌──────────────┐    ┌─────────────────────────────────────┐   │
│  │    ESP32     │───▶│  uart2mqtt.py                       │   │
│  │   (UART)     │    │  ├─ Receives sensor data            │   │
│  │              │    │  ├─ SLAM processing (slam/)         │───┼──▶ MQTT
│  └──────────────┘    │  └─ Publishes pose + map            │   │
│                      └─────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## Quick Start (Docker - Recommended)

### 1. Configure Environment

```bash
cp .env.example .env
nano .env  # Set your MQTT broker IP
```

### 2. Start All Services

```bash
docker compose up -d
```

This starts:
- **mediamtx**: RTSP server on port 8554
- **streamer**: Camera capture → RTSP
- **slam-bridge**: UART → SLAM → MQTT

### 3. Check Logs

```bash
docker compose logs -f slam-bridge
```

### 4. Stop

```bash
docker compose down
```

---

## SLAM Integration

The SLAM algorithm in `slam/` processes encoder and ToF sensor data from the ESP32 to compute the robot's pose and occupancy grid map. 

---

## MQTT Topics

| Topic | Description |
|-------|-------------|
| `robots/{ROBOT_ID}/slam/pose` | Position: `{"x": 1.23, "y": 4.56, "theta": 0.78}` |
| `robots/{ROBOT_ID}/slam/map` | Compressed occupancy grid (base64) |
| `robots/{ROBOT_ID}/telemetry` | Sensor readings (temp, gas, IMU, etc.) |
| `robots/{ROBOT_ID}/status` | Online/offline status |
| `robots/{ROBOT_ID}/command` | Drive commands from backend |

---

## Manual Start (Without Docker)

```bash
# Install dependencies
pip install -r requirements.txt

# Set environment
export MQTT_BROKER=192.168.1.100
export ROBOT_ID=rover-esp32-001

# Run SLAM bridge
python uart2mqtt.py

# In another terminal, start streaming
cd stream && docker compose up -d
```

---

## Hardware Connections

### UART (ESP32 ↔ RPi5)
| ESP32 | RPi5 |
|-------|------|
| TX (GPIO17) | RX (GPIO15) |
| RX (GPIO16) | TX (GPIO14) |
| GND | GND |

### Camera
- USB webcam at `/dev/video0`

---

## Troubleshooting

### UART not working
```bash
sudo raspi-config  # Interface Options → Serial Port → Enable
sudo usermod -a -G dialout $USER
```

### Camera not found
```bash
ls -la /dev/video*
ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 test.jpg
```

### MQTT connection failed
```bash
nc -zv <MQTT_BROKER_IP> 1883
```
