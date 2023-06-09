#!/bin/bash

# TODO: check whether this folder exists
mkdir calibration_file

# Calibrate the robot
docker run --rm --name "ur3e_calibration" \
    --privileged \
    --network "host" \
    --volume "$pwd/calibration_file":/calibration_file \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             timeout 20 roslaunch ur_calibration calibration_correction.launch robot_ip:=150.22.0.41 target_filename:='/calibration_file/ur3e_calibration.yaml'"

docker run -it --name "ur3e_controller" \
    --privileged \
    --restart "always" \
    --network "host" \
    --calibration_file:/calibration_file \
    robotic_base:latest bash
