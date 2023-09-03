#!/bin/bash

# WIFI access point
docker run -d --name "rpi3-wifiap" \
    --restart "always" \
    --privileged \
    --net host \
    -v $(pwd)/config/rpi_wifi/wificfg.json:/cfg/wificfg.json \
    cjimti/iotwifi

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