# rpi5-on-board
Repository with all the important code needed in the Raspberry Pi 5 on the rover (our Edge brain). Also some instructions on how to start it

## Start sending data
To start sending the sensor data to the mqtt broker you only need to start the ESP32 connected to the RPI5, configure the IPs in the uart2mqtt.py file and run the command:
```bash
python3 /home/rpi5/uart2mqtt.py
```

## Live Streaming
Once you have set-up your cloud app (see [cloud-app](https://github.com/TUM-Edge-IoT/cloud-app.git)) and changed the corresponding IPs in the **mediamtx.yml** file and in the frontend you can run the streamer getting into the **stream** folder and running:

```bash
docker compose up -d
```

"Wow! is it that simple?" Yes, my friend, it is. The hard part is getting the Raspberry Pi 5 up and running, but that step should already be done by the time you read this and if not.... good luck.
