# Repository for utilities and tools necessary for setting up hardwares required for robotics subject

## Overview

## For developers, tutors or individuals who are keen
### Setting up the Raspberry Pi (from scratch)
Download the Raspberry Pi Imager and follow the instructions:
- Flash Raspian Lite OS 64bit. Make sure that SD card is at least 16GB
- Before flashing the OS to the SD, ensure the following configurations are met:
  - hostname: robotic
  - Enable SSH with password authentication
  - Username: robotic
  - Password: admin
  - Enable locale settings
  
Insert the SD card into the Raspberry Pi and let it boot, which usually takes around 1 to 2 minutes. After that, connect it to a router and proceed with ssh-ing into it, using the above credentials.
It is also important to have the ip address of the ethernet port to be static. To do so, follow the instructions from this [guide](https://www.makeuseof.com/raspberry-pi-set-static-ip/).
Follow this [guide](https://www.simplilearn.com/tutorials/docker-tutorial/raspberry-pi-docker) to install and configure docker onto the Raspberry Pi. Note that it is not necessary to install docker-compose since we do not rely on it in any of the scripts.
Finally, clone this package to the Pi and put it somewhere convienient for youself.

### How to build base image
`robotic_base` image can only be built on Ubuntu devices (currently supported on MacOS M1 only) and cannot be built on a raspberry pi due to missing dependencies. Therefore, the base image should be built on an Ubuntu device and then transfered onto the Pi for setting up containers.

You can also email the maintainer (Khoa - khoanguyendacdang2198@gmail.com) of this repository for a copy of the base image.
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

##### Disabling wpa_supplicant
To facilitate the wifi access point setup, it is important to disable the **wpa_supplicant** on the Raspberry Pi by using these commands:
```
sudo systemctl mask wpa_supplicant.service
sudo mv /sbin/wpa_supplicant /sbin/no_wpa_supplicant
sudo pkill wpa_supplicant
```
### Configuring Wifi Access Point
It is important to configure the Wifi Access Point before proceeding forward, as it creates a safe point for us to ssh in and debug the unit. It also provides a mean for non - ROS devices to communicate with the ROS network inside the Pi via websocket.
To get started, we need to make the Wifi of each Pi to contain the uuid serial number. Obtain the serial number by running the following command:
```
cat /sys/firmware/devicetree/base/serial-number
```
The output should look something like this:
```
1000000aabbccdd
```
Copy the **last 8 digits of the uuid** and save it somewhere.
After that, navigate to the `config/rpi_wifi` folder and open the file `wificfg.json`. It should look something like this:
```
{
    "dnsmasq_cfg": {
        "address": "/#/192.168.27.1",
        "dhcp_range": "192.168.27.100,192.168.27.150,1h",
        "vendor_class": "set:device,IoT"
    },
    "host_apd_cfg": {
        "ip": "192.168.27.1",
        "ssid": "rpi3_default",
        "wpa_passphrase": "robotics",
        "channel": "6"
    },
    "wpa_supplicant_cfg": {
        "cfg_file": "/etc/wpa_supplicant/wpa_supplicant.conf"
    }
}
```
Replace the `rpi3_default` in the `ssid` field by a combination of the word `robotics_` and the 8 uuid characters that you've obtained from the previous step. The field should look like this:
```
"ssid": "robotics_aabbccdd",
```
Also change the passphrase to `robotics` if necessary.
Save the file and navigate back to the root folder of the repo then run
```
./run_base.sh
```
Double check on your laptop or phone to see if the Wifi is available.

### Run containers
We currently support the following robots:
- UR3 (CB and E series) with the onrobot grippers (RG2 only)
- Hans Cute
- Dobot Magician

#### UR3 series
Please double check your setup before proceeding:
- The following components should be present:
  - UR3 robot
  - Router
  - Raspberry Pi
- They must all be connected to the same network. The UR3 has to be configured with the **External Control URCap** to communicate with ROS. Please follow the instructions [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md) to setup a CB series robot and [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md) to setup an E series robot.

Once that is done, ssh into the Raspberry Pi and run:
E series:
```
./run_ur3e.sh
```
CB series:
```
./run_ur3.sh
```
Please allow at least one minute for all containers to properly start and kick off all of the necessary components.