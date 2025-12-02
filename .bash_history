ping -c 3 google.com
sudo apt update && sudo apt upgrade -y
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
exit
mkdir -p ~/stream
cd stream/
nano docker-compose.yml
ip a
nano mediamtx.yml
docker compose up -d
nano mediamtx.yml
ip a
cat ~/stream/mediamtx.yml
docker logs streamer
cd stream/
nano docker-compose.yml 
docker compose down
docker compose up -d
docker compose down
docker compose up -d
docker compose down
ls
docker compose up -d
cd ..
cat stream/docker-compose.yml 
cat stream/mediamtx.yml 
sudo shutdown now
cd stream/
ls
nano mediamtx.yml 
ls -l /dev/video*
sudo apt update
sudo apt install fswebcam -y
fswebcam --resolution 1280x720 --device /dev/video0 image.jpg
sudo apt update
sudo apt install fswebcam -y
fswebcam --resolution 1280x720 --device /dev/video0 image.jpg
docker compose down
fswebcam --resolution 1280x720 --device /dev/video0 image.jpg
ls -lh image.jpg
docker compose up -d
ip a
docker compose restart mediamtx
docker logs streamer
docker compose down
ls
rm image.jpg 
ls
cat docker-compose.yml 
nano ~/stream/docker-compose.yml
nano ~/stream/mediamtx.yml
docker compose up -d
docker logs streamer
ls
cd stream/
ls
docker compose up -d
cd stream/
docker compose up -d
ip a
cd stream/
ls
d
docker compose down
shutdown now
sudo shutdown now
ip a
cd stream/
ls
cat docker-compose.yml 
cat mediamtx.yml 
ip a
cd stream/
docker compose up -d
cd stream/
docker compose down
sudo shutdown now
sudo apt update
sudo apt install -y python3-pip
pip3 install pyserial paho-mqtt
sudo apt install python3-serial
sudo apt install python3-paho-mqtt
sudo apt install -y jq minicom screen socat
ls -l /dev/serial0 /dev/ttyAMA0 /dev/ttyS0
dmesg | grep -i tty
ls -l /dev/serial0 /dev/ttyAMA*
dmesg | tail -n 80
cat /boot/firmware/cmdline.txt
ls -l /dev/serial0
sudo cat /dev/serial0
sudo usermod -a -G dialout pi
sudo usermod -a -G dialout rpi5
sudo reboot
sudo usermod -a -G dialout rpi5
sudo reboot
cat /boot/firmware/cmdline.txt
sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.bak
sudo nano /boot/firmware/cmdline.txt
sudo reboot
sudo sed -i '/^enable_uart=/d' /boot/firmware/config.txt
echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt
sudo reboot
sudo apt install python3-full -y
python3 -m venv mqtt_env
source mqtt_env/bin/activate
pip install pyserial paho-mqtt
nano uart_to_mqtt.py
source mqtt_env/bin/activate
python3 uart_to_mqtt.py
nano uart_to_mqtt.py
python3 uart_to_mqtt.py
nano uart_to_mqtt.py
rm uart_to_mqtt.py 
nano /home/rpi5/uart2mqtt.py
chmod +x /home/pi/uart2mqtt.py
chmod +x /home/rpi5/uart2mqtt.py
python3 /home/pi/uart2mqtt.py
python3 /home/rpi5/uart2mqtt.py
sudo shutdown -h nom
sudo shutdown -h now
ls
cat mqtt_env
cat mqtt_env/
ls mqtt_env/
cat uasr
cat uart2mqtt.py 
python3 /home/rpi5/uart2mqtt.py
cd stream/
docker compose up -d
ls
nano uart2mqtt.py
cd stream/
docker compose down
cd ..
python3 /home/rpi5/uart2mqtt.py
