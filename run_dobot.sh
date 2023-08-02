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
    --privileged \
    --restart "always" \
    --network "host" \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             roslaunch rosbridge_server rosbridge_websocket.launch"

# Start the robot
docker run -d --name "dobot_controller" \
    --privileged \
    --restart "always" \
    --network "host" \
    --mount type=bind,source=/dev/,target=/dev/ \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /db_ws/devel/setup.bash && \
             sleep 15 && \
             roslaunch dobot_magician_driver dobot_magician.launch"