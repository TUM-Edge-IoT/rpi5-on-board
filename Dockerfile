# =============================================================================
# RPi5 SLAM Bridge - Python Container
# =============================================================================
# This Dockerfile builds the container for uart2mqtt.py with SLAM processing
# =============================================================================

FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy SLAM module
COPY slam/ ./slam/

# Copy main script
COPY uart2mqtt.py .

# Run the UART to MQTT bridge with SLAM
CMD ["python", "-u", "uart2mqtt.py"]
