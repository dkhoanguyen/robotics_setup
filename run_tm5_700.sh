#!/bin/bash

POSITIONAL_ARGS=()

while [[ $# -gt 0 ]]; do
  case $1 in
    -r|--robot_ip)
      ROBOT_IP="$2"
      shift # past argument
      shift # past value
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
    *)
      POSITIONAL_ARGS+=("$1") # save positional arg
      shift # past argument
      ;;
  esac
done

set -- "${POSITIONAL_ARGS[@]}" # restore positional parameters
echo "Robot IP = ${ROBOT_IP}"

# Load calibration file and start the robot
docker run -d --name "tm5_700_controller" \
    --tty \
    --privileged \
    --restart "always" \
    --network "host" \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e ROS_IP="192.168.27.1" \
    dkhoanguyen/robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /tm_ws/devel/setup.bash && \
             sleep 15 && \
             rosrun tm_driver tm_driver ${ROBOT_IP}"