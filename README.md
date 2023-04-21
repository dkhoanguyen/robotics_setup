# Repository for utilities and tools necessary for setting up hardwares required for robotics subject

## Overview

## For developers, tutors or individuals who are keen
### Setting up the Raspberry Pi
Download the Raspberry Pi Imager and follow the instructions:
- Flash Raspian Lite OS 64bit. Make sure that SD card is at least 16GB
- Before flashing the OS to the SD, ensure the following configurations are met:
  - hostname: robotic
  - Enable SSH with password authentication
  - Username: robotic
  - Password: admin
  - Enable locale settings
### How to build base image
`robotic_base` image can only be built on Ubuntu devices (currently supported on MacOS M1 only) and cannot be built on a raspberry pi due to missing dependencies. Therefore, the base image should be built on an Ubuntu device and then transfered onto the Pi for setting up containers.

You can also ask the maintainer (Khoa - khoanguyendacdang2198@gmail.com) of this repository for a copy of the base image.
To build the base image, follow the instructions:

##### On the Ubuntu device:
Create a folder `built_images` and build the image using the `build.sh` script
```
mkdir built_images
./build.sh
```

Save the base image into a file in `built_images`
```
docker save -o built_images/robotic_base robotic_base:latest
```

Wait for the process to finish then copy that image onto the pi using `scp`
```
scp built_images/robotic_base robotic@<pi-ip-address>:/home/robotic/robotic_base
```
##### On the Raspberry Pi:
Navigate to `/home/robotic/` and make sure that the file exists. Then run this command to load the image onto docker:
```
docker load -i robotic_base
```

Please make sure that there is sufficient space on the Pi for this operation. Once that done then the image should be available for use. You can double-check using `docker image ls -a`

### Run containers
Simply execute
```
./run.sh
```
This script should execute the following operations:
- Create a `calibration_file` folder for storing the calibration result of a specific UR robot
- Perform a robot calibration and store the calibration result in the `calibration_file` folder
- Start the controller with the provided calibration result