FROM arm64v8/ros:noetic-ros-base as configure_stage

# Setup basic
RUN apt update && apt install -y \
    git \
    gcc \
    curl \
    wget \
    ros-noetic-tf \
    ros-noetic-tf2-tools \
    python3-catkin-tools \
    python3-pip

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
RUN pip3 install pymodbus==2.5.3
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

# Configure Dobot ws
RUN mkdir -p db_ws/src && \
    cd db_ws && \
    catkin init && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# dobot_magician_driver
RUN apt install -y g++ git autogen autoconf build-essential cmake graphviz \
                 libboost-dev libboost-test-dev libgtest-dev libtool \
                 python3-sip-dev doxygen python3-sphinx pkg-config \
                 python3-sphinx-rtd-theme

RUN git clone https://github.com/crayzeewulf/libserial && \
    cd libserial && \
    ./compile.sh && \
    cd build && \
    make install && \
    apt install -y libserialport0 libusb-1.0-0-dev

RUN cd db_ws/src && \
    git clone https://github.com/gapaul/dobot_magician_driver.git

# Build db_ws
RUN bin/bash -c "source /opt/ros/noetic/setup.bash && \
                 cd db_ws && \
                 catkin build"

# Configure Dobot ws
RUN mkdir -p hans_ws/src && \
    cd hans_ws && \
    catkin init && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    cd src && \ 
    git clone https://github.com/dkhoanguyen/hans-cute-driver

# Has to catkin_make here instead of build due to serial_port not being a catkin package
RUN bin/bash -c "source /opt/ros/noetic/setup.bash && \
                 cd hans_ws && \
                 catkin_make"

# Omron Robots
RUN mkdir -p omron_ws/src && \
    cd omron_ws && \
    catkin init && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    cd src && \ 
    git clone -b noetic https://github.com/TechmanRobotInc/tmr_ros1.git && \
    cd ../ && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y --os=ubuntu:focal -y 

# # Build omron_ws
# RUN bin/bash -c "source /opt/ros/noetic/setup.bash && \
#                  cd omron_ws && \
#                  catkin build"

# # Add sensors support
# RUN apt update && apt install -y \
#     ros-noetic-realsense2-camera \
#     ros-noetic-usb-cam


COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh

RUN mkdir calibration_file

FROM scratch
COPY --from=configure_stage / /
