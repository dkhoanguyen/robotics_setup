#!/bin/bash

if [ "$#" -lt 4 ]; then
  echo "Usage: $0 <--robot_ip> <val> <--gripper_ip> <val> "
  exit 1
fi

# TODO: Cater for the use of gripper
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


file_path="calibration_file/ur3_calibration.yaml"
# TODO: check whether this folder exists
if [ -e "$file_path" ]; then
    echo "File exists in the folder."
else
    echo "Performing calibration"
    mkdir calibration_file

    # Calibrate the robot
    docker run --rm --name "ur3_calibration" \
    --tty \
    --privileged \
    --network "host" \
    --volume "$(pwd)"/calibration_file:/calibration_file \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             timeout 20 roslaunch ur_calibration calibration_correction.launch \
             robot_ip:=${ROBOT_IP} target_filename:='/calibration_file/ur3_calibration.yaml'"
fi

# Load calibration file and start the robot
docker run -d --name "ur3_controller" \
    --tty \
    --privileged \
    --restart "always" \
    --network "host" \
    --mount type=bind,source="$(pwd)"/calibration_file,target=/calibration_file \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e ROS_IP="192.168.27.1" \
    robotic_base:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
             sleep 15 && \
             roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=${ROBOT_IP} \
             kinematics_config:='/calibration_file/ur3_calibration.yaml'"

# # Start gripper
# docker run -d --name "gripper_hw_interface" \
#     --tty \
#     --privileged \
#     --restart "always" \
#     --network "host" \
#     -e ROS_MASTER_URI="http://localhost:11311" \
#     -e ROS_IP="192.168.27.1" \
#     robotic_base:latest \
#     bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
#              sleep 30 && \
#              roslaunch onrobot_rg_control bringup.launch gripper:=rg2 ip:=${GRIPPER_IP}"

# docker run -d --name "gripper_controller" \
#     --tty \
#     --privileged \
#     --restart "always" \
#     --network "host" \
#     -e ROS_MASTER_URI="http://localhost:11311" \
#     -e ROS_IP="192.168.27.1" \
#     robotic_base:latest \
#     bash -c "source /opt/ros/noetic/setup.bash && source /ur_ws/devel/setup.bash && \
#              sleep 45 && \
#              rosrun onrobot_rg_control OnRobotRGSimpleControllerServer.py"

