#!/usr/bin/env python3
# uart_to_mqtt.py — read newline JSON from /dev/serial0 and publish to robots/<device>/telemetry

import serial
import json
import time
import paho.mqtt.client as mqtt
import os
import sys
from datetime import datetime

# --- config ---
#BROKER_IP = os.getenv("MQTT_BROKER", "172.20.10.4")  # RPi3 IP
BROKER_IP = os.getenv("MQTT_BROKER", "10.145.57.230")  # RPi3 IP
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))
UART_DEVICE = os.getenv("UART_DEVICE", "/dev/serial0")
UART_BAUD = int(os.getenv("UART_BAUD", "115200"))
MQTT_QOS = 1

# --- open UART ---
try:
    ser = serial.Serial(port=UART_DEVICE, baudrate=UART_BAUD, timeout=1)
except Exception as e:
    print("Failed to open serial port", UART_DEVICE, e)
    sys.exit(1)

# --- mqtt client ---
client = mqtt.Client()
# If your broker requires auth, set username/pass:
# client.username_pw_set("backend", "backend_password")

def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] connected rc={rc} to {BROKER_IP}:{BROKER_PORT}")

def on_disconnect(client, userdata, rc):
    print(f"[MQTT] disconnected rc={rc}")

client.on_connect = on_connect
client.on_disconnect = on_disconnect

try:
    client.connect(BROKER_IP, BROKER_PORT, keepalive=60)
except Exception as e:
    print("MQTT connect error (will retry in background):", e)

client.loop_start()  # start background network loop / auto-reconnect

print("Listening for UART messages and publishing to MQTT...")
print(f"UART: {UART_DEVICE}@{UART_BAUD}  Broker: {BROKER_IP}:{BROKER_PORT}")

def normalize_payload_for_db(payload_obj):
    """
    Optional: normalize keys from ESP32 JSON to backend expected keys.
    For example, convert temp_c -> temperature, distance_cm -> distance, co2_ppm -> co2.
    Modify as needed for your firmware's exact field names.
    """
    normalized = {}
    # Temperature
    if "temperature" in payload_obj:
        normalized["temperature"] = payload_obj["temperature"]
    elif "temp_c" in payload_obj:
        normalized["temperature"] = payload_obj["temp_c"]
    elif "t" in payload_obj:
        normalized["temperature"] = payload_obj["t"]

    # Distance
    if "distance" in payload_obj:
        normalized["distance"] = payload_obj["distance"]
    elif "distance_cm" in payload_obj:
        normalized["distance"] = payload_obj["distance_cm"]

    # CO2
    if "co2" in payload_obj:
        normalized["co2"] = payload_obj["co2"]
    elif "co2_ppm" in payload_obj:
        normalized["co2"] = payload_obj["co2_ppm"]

    # MQ2 LPG / CO
    if "mq2_lpg" in payload_obj:
        normalized["mq2_lpg"] = payload_obj["mq2_lpg"]
    elif "mq2_lpg_ppm" in payload_obj:
        normalized["mq2_lpg"] = payload_obj["mq2_lpg_ppm"]

    if "mq2_co" in payload_obj:
        normalized["mq2_co"] = payload_obj["mq2_co"]
    elif "mq2_co_ppm" in payload_obj:
        normalized["mq2_co"] = payload_obj["mq2_co_ppm"]

    # keep everything else as-is if you want raw JSON too:
    normalized["_raw"] = payload_obj
    # add timestamp from Pi (optional)
    normalized["_received_at"] = datetime.utcnow().isoformat() + "Z"
    return normalized

while True:
    try:
        line = ser.readline()
        if not line:
            continue
        text = line.decode("utf-8", errors="replace").strip()
        if not text:
            continue

        print("[UART RX]", text)

        # parse JSON (if the payload isn't JSON this will throw)
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            # fallback: publish raw string to a generic topic
            print("[WARN] Invalid JSON received; publishing raw string to robots/unknown/telemetry_raw")
            client.publish("robots/unknown/telemetry_raw", text, qos=MQTT_QOS)
            continue

        # derive device id: prefer data.device else fallback to 'unknown'
        device = data.get("device") if isinstance(data, dict) else None
        if not device:
            # if your payload has other id fields, check them here
            device = data.get("id") if isinstance(data, dict) else None
        if not device:
            device = "unknown"

        # prepare payload to publish to backend
        # Option A (recommended): publish normalized JSON that matches backend fields:
        publish_obj = normalize_payload_for_db(data)

        # Option B (alternative): publish raw payload exactly as ESP32 sent
        # publish_obj = data

        topic = f"robots/{device}/telemetry"
        payload_str = json.dumps(publish_obj, separators=(",", ":"), ensure_ascii=False)

        # publish (with qos=1)
        result = client.publish(topic, payload_str, qos=MQTT_QOS)
        # optionally inspect result: result.wait_for_publish() or check rc
        print(f"[MQTT PUB] {topic} -> {payload_str}")

    except Exception as e:
        print("Unexpected error in main loop:", e)
        time.sleep(0.5)
