#!/usr/bin/env python3
import serial
import json
import time
import paho.mqtt.client as mqtt
import os
import sys
import socket
from datetime import datetime

# --- CONFIG ---
BROKER_IP = os.getenv("MQTT_BROKER", "10.145.57.230")
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))
UART_DEVICE = os.getenv("UART_DEVICE", "/dev/serial0")
UART_BAUD = int(os.getenv("UART_BAUD", "115200"))
MQTT_QOS = 1

# --- HELPER: GET LOCAL IP ---
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

# --- HELPER: NORMALIZE DATA ---
def normalize_payload_for_db(payload_obj):
    normalized = {}

    # 1. Temperature
    if "temperature" in payload_obj: normalized["temperature"] = payload_obj["temperature"]
    elif "temp_c" in payload_obj: normalized["temperature"] = payload_obj["temp_c"]
    elif "t" in payload_obj: normalized["temperature"] = payload_obj["t"]

    # 2. Distance
    if "distance" in payload_obj: normalized["distance"] = payload_obj["distance"]
    elif "distance_cm" in payload_obj: normalized["distance"] = payload_obj["distance_cm"]

    # 3. CO2
    if "co2" in payload_obj: normalized["co2"] = payload_obj["co2"]
    elif "co2_ppm" in payload_obj: normalized["co2"] = payload_obj["co2_ppm"]

    # 4. MQ2
    if "mq2_lpg" in payload_obj: normalized["mq2_lpg"] = payload_obj["mq2_lpg"]
    elif "mq2_lpg_ppm" in payload_obj: normalized["mq2_lpg"] = payload_obj["mq2_lpg_ppm"]

    if "mq2_co" in payload_obj: normalized["mq2_co"] = payload_obj["mq2_co"]
    elif "mq2_co_ppm" in payload_obj: normalized["mq2_co"] = payload_obj["mq2_co_ppm"]

    # Add timestamp and raw data
    normalized["_raw"] = payload_obj
    normalized["_received_at"] = datetime.utcnow().isoformat() + "Z"
    return normalized

# --- 1. OPEN SERIAL ---
try:
    ser = serial.Serial(port=UART_DEVICE, baudrate=UART_BAUD, timeout=1)
    print(f"[INIT] Serial opened on {UART_DEVICE}")
except Exception as e:
    print(f"[FATAL] Failed to open serial: {e}")
    sys.exit(1)

# --- 2. DISCOVERY PHASE (Wait for ESP32 ID) ---
ROBOT_ID = None
print("[INIT] Waiting for data from ESP32 to identify myself...")

while ROBOT_ID is None:
    try:
        line = ser.readline()
        if not line: continue
        text = line.decode("utf-8", errors="replace").strip()
        try:
            data = json.loads(text)
            if "device" in data: ROBOT_ID = data["device"]
            elif "id" in data: ROBOT_ID = data["id"]

            if ROBOT_ID:
                print(f"✅ IDENTIFIED! I am: {ROBOT_ID}")
        except:
            pass
    except Exception as e:
        print("Waiting...", e)
        time.sleep(1)

# --- 3. MQTT SETUP ---
client = mqtt.Client()

# --- LAST WILL AND TESTAMENT ---
will_topic = f"robots/{ROBOT_ID}/status"
will_payload = json.dumps({"status": "offline", "ip_address": None})
client.will_set(will_topic, will_payload, qos=1, retain=True)

def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] Connected rc={rc}")
    
    # Subscribe to commands
    cmd_topic = f"robots/{ROBOT_ID}/command"
    client.subscribe(cmd_topic)
    print(f"[MQTT] Listening for commands on: {cmd_topic}")

    # --- ANNOUNCE ONLINE STATUS ---
    my_ip = get_local_ip()
    status_payload = {
        "status": "online",
        "ip_address": my_ip,
        "video_path": "cam",
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }
    client.publish(f"robots/{ROBOT_ID}/status", json.dumps(status_payload), qos=1, retain=True)
    print(f"[SYSTEM] Announced ONLINE: {my_ip}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        print(f"[CMD RX] {payload}")
        data = json.loads(payload)
        if "command" in data:
            cmd = data["command"] + "\n"
            ser.write(cmd.encode())
    except Exception as e:
        print(f"Cmd Error: {e}")

client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(BROKER_IP, BROKER_PORT, 60)
    client.loop_start()
except Exception as e:
    print(f"[FATAL] MQTT Connection failed: {e}")

# --- 4. MAIN TELEMETRY LOOP ---
print("System Online. Forwarding Data...")

# !!! THIS TRY BLOCK WAS MISSING IN YOUR CODE !!!
try:
    while True:
        try:
            line = ser.readline()
            if not line: continue
            text = line.decode("utf-8", errors="replace").strip()
            if not text: continue

            # Parse JSON
            try:
                data = json.loads(text)
            except json.JSONDecodeError:
                print(f"[WARN] Invalid JSON: {text}")
                continue

            # --- NORMALIZE DATA ---
            publish_obj = normalize_payload_for_db(data)

            # Publish
            topic = f"robots/{ROBOT_ID}/telemetry"
            payload_str = json.dumps(publish_obj)

            client.publish(topic, payload_str, qos=1)
            print(f"[PUB] {topic} -> {payload_str}")

        except Exception as e:
            print("Loop Error:", e)
            time.sleep(1)

except KeyboardInterrupt:
    print("\n[SYSTEM] Stopping... Sending OFFLINE status.")

    # 1. Manually publish OFFLINE status
    offline_payload = json.dumps({
        "status": "offline",
        "ip_address": None
    })
    info = client.publish(f"robots/{ROBOT_ID}/status", offline_payload, qos=1, retain=True)

    # 2. Wait for the message to leave the network card
    info.wait_for_publish()

    # 3. Disconnect cleanly
    client.disconnect()
    client.loop_stop()
    print("[SYSTEM] Goodbye.")
