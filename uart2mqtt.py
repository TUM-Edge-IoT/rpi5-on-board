#!/usr/bin/env python3
import serial
import json
import time
import paho.mqtt.client as mqtt
import os
import sys
import socket
from datetime import datetime, timezone
import base64
import zlib
import numpy as np
import signal 

# =============================================================================
# CONFIG
# =============================================================================
#BROKER_IP   = os.getenv("MQTT_BROKER", "10.110.117.139") 
BROKER_IP   = os.getenv("MQTT_BROKER", "172.20.10.3")
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))

UART_DEVICE = os.getenv("UART_DEVICE", "/dev/ttyAMA0")
UART_BAUD   = int(os.getenv("UART_BAUD", "115200"))

ROBOT_ID = os.getenv("ROBOT_ID", "rover-esp32-001")
MQTT_QOS = 1

SLAM_PUBLISH_INTERVAL = float(os.getenv("SLAM_PUBLISH_INTERVAL", "1.0"))
last_slam_publish_time = 0.0

# =============================================================================
# HELPERS
# =============================================================================
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip

def publish_offline(reason=None):
    payload = {
        "status": "offline",
        "reason": reason,
        "timestamp": datetime.now(timezone.utc).isoformat()
    }
    try:
        # We capture the info object to wait for it later
        info = client.publish(f"robots/{ROBOT_ID}/status",
                       json.dumps(payload),
                       qos=1,
                       retain=True)
        return info
    except Exception:
        return None

# =============================================================================
# SLAM ENGINE
# =============================================================================
try:
    from slam.slam_engine import SlamEngine
    slam = SlamEngine()
    slam_enabled = True
    print("[INIT] SLAM engine initialized successfully")
except Exception as e:
    slam = None
    slam_enabled = False
    print(f"[WARN] SLAM engine disabled: {e}")

# =============================================================================
# MQTT SETUP
# =============================================================================
client_id = f"{ROBOT_ID}"

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, clean_session=True)

# LWT (Last Will) - Backup in case of power failure/crash
will_payload = json.dumps({
    "status": "offline",
    "ip_address": None,
    "timestamp": datetime.now(timezone.utc).isoformat()
})

client.will_set(
    f"robots/{ROBOT_ID}/status",
    will_payload,
    qos=1,
    retain=True
)

def on_connect(client, userdata, flags, reason_code, properties=None):
    rc = reason_code.value if hasattr(reason_code, 'value') else reason_code
    print(f"[MQTT] Connected rc={rc}")
    
    if rc != 0:
        print(f"[FATAL] Connection rejected with code {rc}")
        return

    cmd_topic = f"robots/{ROBOT_ID}/command"
    client.subscribe(cmd_topic)

    status_payload = {
        "status": "online",
        "ip_address": get_local_ip(),
        "video_path": "cam",
        "timestamp": datetime.now(timezone.utc).isoformat()
    }

    client.publish(
        f"robots/{ROBOT_ID}/status",
        json.dumps(status_payload),
        qos=1,
        retain=True
    )

    print(f"[SYSTEM] ONLINE announced as {ROBOT_ID}")

def on_message(client, userdata, msg):
    global ser
    if ser is None:
        print("[UART] Not ready, dropping command")
        return

    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)
        if "command" in data:
            ser.write((payload + "\n").encode())
            print(f"[UART TX] {payload}")
    except Exception as e:
        print(f"[CMD ERROR] {e}")

client.on_connect = on_connect
client.on_message = on_message

try:
    print(f"[MQTT] Connecting to {BROKER_IP}:{BROKER_PORT}...")
    client.connect(BROKER_IP, BROKER_PORT, 60)
    client.loop_start()
except Exception as e:
    print(f"[FATAL] MQTT connection failed: {e}")
    sys.exit(1)

# =============================================================================
# SERIAL (RETRY-BASED)
# =============================================================================
ser = None

def try_open_serial():
    global ser
    try:
        ser = serial.Serial(UART_DEVICE, UART_BAUD, timeout=1)
        print(f"[UART] Opened {UART_DEVICE}")
    except Exception as e:
        ser = None
        # print(f"[UART] Unavailable: {e}") 

# =============================================================================
# NORMALIZATION
# =============================================================================
def normalize_payload_for_db(payload):
    out = {}
    if "temperature" in payload: out["temperature"] = payload["temperature"]
    elif "temp_c" in payload: out["temperature"] = payload["temp_c"]

    if "distance" in payload: out["distance"] = payload["distance"]
    elif "distance_cm" in payload: out["distance"] = payload["distance_cm"]

    out["_raw"] = payload
    out["_received_at"] = datetime.now(timezone.utc).isoformat()
    return out

# =============================================================================
# GRACEFUL SHUTDOWN HANDLER (SIGTERM/SIGINT)
# =============================================================================
def graceful_shutdown(signum, frame):
    print(f"\n[SYSTEM] Caught signal {signum}. Shutting down gracefully...")
    
    info = publish_offline("docker stop")
    
    if info:
        info.wait_for_publish(timeout=2)
        
    client.disconnect()
    client.loop_stop()
    print("[SYSTEM] Goodbye.")
    sys.exit(0)

signal.signal(signal.SIGINT, graceful_shutdown)  # Ctrl+C
signal.signal(signal.SIGTERM, graceful_shutdown) # Docker stop

# =============================================================================
# MAIN LOOP
# =============================================================================
print("[SYSTEM] SLAM Bridge running")

while True:
    if ser is None:
        try_open_serial()
        time.sleep(2)
        continue

    try:
        line = ser.readline()
        if not line:
            continue

        text = line.decode("utf-8", errors="replace").strip()
        print(f"[RAW DATA]:{text}")
        if not text:
            continue

        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            continue
        if "timestamp_ms" in data:
            data["ts"] = data["timestamp_ms"]
        if "encoders" in data:
            data["enc"] = {"l": data["encoders"].get("e1",0),"r":data["encoders"].get("e2",0)}
        if "tof" in data and isinstance(data["tof"], dict):
            for k, v in data["tof"].items():
                try:
                    data["tof"][k]=int(v)
                except (ValueError, TypeError):
                    data["tof"][k]=0

        is_slam = "imu" in data
        is_telem = "device" in data or "temperature" in data

        if is_slam and slam_enabled:
            slam_out = slam.process(data)
            if not slam_out:
                continue

            now = time.time()
            if now - last_slam_publish_time < SLAM_PUBLISH_INTERVAL:
                continue
            last_slam_publish_time = now

            client.publish(
                f"robots/{ROBOT_ID}/slam/pose",
                json.dumps(slam_out["pose"]),
                qos=1
            )

            raw_map = slam_out["map"]
            display = np.zeros_like(raw_map, dtype=np.uint8)
            display[raw_map < 0] = 1
            display[raw_map > 0] = 100

            encoded = base64.b64encode(
                zlib.compress(display.tobytes())
            ).decode()

            client.publish(
                f"robots/{ROBOT_ID}/slam/map",
                json.dumps({
                    "resolution": 0.05,
                    "size": raw_map.shape,
                    "data": encoded
                }),
                qos=1
            )
            print(f"[SLAM] Map updated")

        if is_telem:
            payload = normalize_payload_for_db(data)
            client.publish(
                f"robots/{ROBOT_ID}/telemetry",
                json.dumps(payload),
                qos=1
            )
            print(f"[TELEM] Sent data")

    except serial.SerialException:
        print("[UART] Lost connection")
        ser = None
        time.sleep(2)
