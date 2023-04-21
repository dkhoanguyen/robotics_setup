#!/bin/bash

# TODO: check whether this folder exists
mkdir calibration_file

# Calibrate the robot
docker run -d -rm --name "ur3e_calibration" \
    --privileged \
    --network "host" \
    --calibration_file:/calibration_file \
    robotic_base:latest \
    "/bin/bash -c source /ur_ws/devel/setup.bash &&\
    timeout 20 roslaunch ur_calibration calibration_correction.launch robot_ip:=150.22.0.41 target_filename:='/calibration_file/ur3e_calibration.yaml'"

docker run -it --name "ur3e_controller" \
    --privileged \
    --restart "always" \
    --network "host" \
    --calibration_file:/calibration_file \
    robotic_base:latest bash
