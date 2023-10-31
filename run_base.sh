#!/bin/bash

if grep -q 'BCM' /proc/cpuinfo && grep -q 'Raspberry Pi' /sys/firmware/devicetree/base/model; then
    echo "This is a Raspberry Pi. Proceeding with the command."
    # Configure RPI Wifi access point
    # Prerequisites
    echo 'denyinterfaces wlan0' >> /etc/dhcpcd.conf
    cd rpi_wifi_ap/

    # Build docker image
    docker build . --tag rpi3-wifiap

    # Start the access point
    docker run -d --name "rpi3-wifiap" \
        --restart "always" \
        --tty \
        --privileged \
        --cap-add=NET_ADMIN \
        --network=host  \
        --volume "$(pwd)"/confs/hostapd_confs/robotics.conf:/etc/hostapd/hostapd.conf \
        --label=com.centurylinklabs.watchtower.enable=false \
        rpi3-wifiap
else
    echo "This is not a Raspberry Pi. Aborting the command."
fi

# Rosbridge server
docker run -d --name "rosbridge" \
    --tty \
    --privileged \
    --restart "always" \
    --network "host" \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e ROS_IP="192.168.27.1" \
    dkhoanguyen/robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             roslaunch rosbridge_server rosbridge_websocket.launch"

# Watchtower for monitoring updates
docker run -d --name "watchtower" \
  --tty \
  --privileged \
  --restart "always" \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -e WATCHTOWER_CLEANUP=true \
  -e WATCHTOWER_INCLUDE_RESTARTING=true \
  -e WATCHTOWER_HTTP_API_TOKEN=robotics \
  -e WATCHTOWER_HTTP_API_PERIODIC_POLLS=true \
  -p 8080:8080 \
  --label=com.centurylinklabs.watchtower.enable=false \
  dkhoanguyen/watchtower:latest --interval 10 --http-api-update --port 8080 --update-on-startup