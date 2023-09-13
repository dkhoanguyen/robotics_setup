#!/bin/bash

# Configure RPI Wifi access point
# Prerequisites
echo 'denyinterfaces wlan0' >> /etc/dhcpcd.conf
cd rpi_wifi_ap/

# Build docker image
docker build . --tag rpi_wifi_ap

# Start the access point
docker run -d --name "rpi3-wifiap" \
    --restart "always" \
    --tty \
    --privileged \
    --cap-add=NET_ADMIN \
    --network=host  rpi3-wifiap

# Rosbridge server
docker run -d --name "rosbridge" \
    --tty \
    --privileged \
    --restart "always" \
    --network "host" \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e ROS_IP="192.168.27.1" \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             roslaunch rosbridge_server rosbridge_websocket.launch"

# A utils dev container for testing purposes
docker run -d --name "dev_container" \
    --privileged \
    --restart "always" \
    --network "host" \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e ROS_IP="192.168.27.1" \
    robotic_base:latest \
    sleep infinity