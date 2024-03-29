#!/bin/bash

# POSITIONAL_ARGS=()

# while [[ $# -gt 0 ]]; do
#   case $1 in
#     -r|--gripper_ip)
#       GRIPPER_IP="$2"
#       shift # past argument
#       shift # past value
#       ;;
#     -*|--*)
#       echo "Unknown option $1"
#       exit 1
#       ;;
#     *)
#       POSITIONAL_ARGS+=("$1") # save positional arg
#       shift # past argument
#       ;;
#   esac
# done

# set -- "${POSITIONAL_ARGS[@]}" # restore positional parameters
# echo "Gripper IP = ${GRIPPER_IP}"

# Start gripper
docker run -d --name "gripper_hw_interface" \
    --tty \
    --privileged \
    --restart "always" \
    --network "host" \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e ROS_IP="192.168.27.1" \
    dkhoanguyen/robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /tm_ws/devel/setup.bash && \
             sleep 15 && \
             roslaunch onrobot_rg_joint_position_control bringup_server.launch"