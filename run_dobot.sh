#!/bin/bash

# Start the robot
docker run -d --name "dobot_controller" \
    --tty \
    --privileged \
    --restart "always" \
    --network "host" \
    --mount type=bind,source=/dev/,target=/dev/ \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e ROS_IP="192.168.27.1" \
    dkhoanguyen/robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /db_ws/devel/setup.bash && \
             sleep 15 && \
             roslaunch dobot_magician_driver dobot_magician.launch"