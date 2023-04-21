FROM arm64v8/ros:noetic-ros-base as configure_stage

# Setup basic
RUN apt update && apt install -y \
    git \
    gcc \
    curl \
    wget

RUN apt install -y \
    ros-noetic-tf \
    ros-noetic-tf2-tools \
    python3-catkin-tools

# Install ROS bridge
RUN apt update && apt install -y ros-noetic-rosbridge-suite

# Expose port for bridge
EXPOSE 9090
EXPOSE 11311

# Configure ur_ws
RUN mkdir -p ur_ws/src && \
    cd ur_ws && \
    catkin init && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# ur_ros_driver
RUN cd ur_ws && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver && \
    git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot && \
    apt update -qq && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# onrobot gripper
RUN cd ur_ws/src && \
    git clone https://github.com/takuya-ki/onrobot.git --depth 1 && \
    git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git --depth 1 && \
    cd ../ && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y --os=ubuntu:focal -y && \
    apt install -y ros-noetic-ros-control ros-noetic-ros-controllers

# Build ur_ws
RUN bin/bash -c "source /opt/ros/noetic/setup.bash && \
                 cd ur_ws && \
                 catkin build"

COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh

RUN mkdir calibration_file

FROM scratch
COPY --from=configure_stage / /
