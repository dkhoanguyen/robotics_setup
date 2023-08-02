#!/bin/bash

if [ "$#" -lt 4 ]; then
  echo "Usage: $0 <--robot_ip> <val> <--gripper_ip> <val> "
  exit 1
fi

# WIFI access point
docker run -d --name "rpi3-wifiap" \
    -e SSID="rpi3_default" \
    -e PASSWORD="robotic" \
    --restart "always" \
    --privileged \
    --pid=host \
    --net=host jasonhillier/rpi3-wifiap

# Rosbridge server
docker run -d --name "rosbridge" \
    --privileged \
    --restart "always" \
    --network "host" \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             roslaunch rosbridge_server rosbridge_websocket.launch"

# Load calibration file and start the robot
docker run -d --name "ur3_controller" \
    --privileged \
    --restart "always" \
    --network "host" \
    --mount type=bind,source="$(pwd)"/calibration_file,target=/calibration_file \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             sleep 15 && \
             roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=${ROBOT_IP} \
             kinematics_config:='/calibration_file/ur3_calibration.yaml'"

# Start gripper
docker run -d --name "gripper_hw_interface" \
    --privileged \
    --restart "always" \
    --network "host" \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             sleep 30 && \
             roslaunch onrobot_rg_control bringup.launch gripper:=rg2 ip:=${GRIPPER_IP}"

docker run -d --name "gripper_controller" \
    --privileged \
    --restart "always" \
    --network "host" \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             sleep 45 && \
             rosrun onrobot_rg_control OnRobotRGSimpleControllerServer.py"

