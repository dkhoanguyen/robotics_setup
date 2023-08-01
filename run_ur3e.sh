#!/bin/bash

if [ "$#" -lt 4 ]; then
  echo "Usage: $0 <--robot_ip> <val> <--gripper_ip> <val> "
  exit 1
fi


POSITIONAL_ARGS=()

while [[ $# -gt 0 ]]; do
  case $1 in
    -r|--robot_ip)
      ROBOT_IP="$2"
      shift # past argument
      shift # past value
      ;;
    -r|--gripper_ip)
      GRIPPER_IP="$2"
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
echo "Gripper IP = ${GRIPPER_IP}"

# Grab RPI serial number
SERIAL=$(cat /sys/firmware/devicetree/base/serial-number)
SSID="rpi_${SERIAL: -8}"

# WIFI access point
docker run -d --name "rpi3-wifiap" \
    -e SSID="${SSID}" \
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


file_path="calibration_file/ur3e_calibration.yaml"
# TODO: check whether this folder exists
if [ -e "$file_path" ]; then
    echo "File exists in the folder."
else
    echo "Performing calibration"
    mkdir calibration_file

    # Calibrate the robot
    docker run --rm --name "ur3e_calibration" \
    --privileged \
    --network "host" \
    --volume "$(pwd)"/calibration_file:/calibration_file \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             timeout 20 roslaunch ur_calibration calibration_correction.launch \
             robot_ip:=${ROBOT_IP} target_filename:='/calibration_file/ur3e_calibration.yaml'"
fi

# Load calibration file and start the robot
docker run -d --name "ur3e_controller" \
    --privileged \
    --restart "always" \
    --network "host" \
    --mount type=bind,source="$(pwd)"/calibration_file,target=/calibration_file \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=${ROBOT_IP} \
             kinematics_config:='/calibration_file/ur3e_calibration.yaml'"

# Start gripper
docker run -d --name "gripper_hw_interface" \
    --privileged \
    --restart "always" \
    --network "host" \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             roslaunch onrobot_rg_control bringup.launch gripper:=rg2 ip:=${GRIPPER_IP}"

docker run -d --name "gripper_controller" \
    --privileged \
    --restart "always" \
    --network "host" \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             rosrun onrobot_rg_control OnRobotRGSimpleControllerServer.py"

