# AutoWLAN
#### by @guille_hartek on [Follow the White Rabbit](https://fwhibbit.es)

![](cover.png)

This project will allow you run a portable access point on a Raspberry Pi making use of _Docker_ containers. 

Further reference and explanations: 

<https://fwhibbit.es/en/automatic-access-point-with-docker-and-raspberry-pi-zero-w>

Tested on Raspberry Pi Zero W.

### Access point configurations
You can customize the network password and other configurations on files at _confs/hostapd_confs/_. You can also add your own _hostapd_ configuration files here. 

### Management using plain docker
Add _--rm_ for volatile containers. 
##### Create and run a container with default (Open) configuration (stop with Ctrl+C)
~~~
docker run --name autowlan_open --cap-add=NET_ADMIN --network=host  autowlan
~~~
##### Create and run a container with WEP configuration (stop with Ctrl+C)
~~~
docker run --name autowlan_wep --cap-add=NET_ADMIN --network=host -v $(pwd)/confs/hostapd_confs/wep.conf:/etc/hostapd/hostapd.conf autowlan
~~~
##### Create and run a container with WPA2 configuration (stop with Ctrl+C)
~~~
docker run --name autowlan_wpa2 --cap-add=NET_ADMIN --network=host -v $(pwd)/confs/hostapd_confs/wpa2.conf:/etc/hostapd/hostapd.conf autowlan
~~~
##### Stop a running container
~~~
docker stop autowlan_{open|wep|wpa2}
~~~


### Management using docker-compose
##### Create and run container (stop with Ctrl+C)
~~~
docker-compose -f <fichero_yml> up
~~~
##### Create and run container in the background
~~~
docker-compose -f <fichero_yml> up  -d
~~~
##### Stop a container in the background
~~~
docker-compose -f <fichero_yml> down
~~~
##### Read logs of a container in the background
~~~
docker-compose -f <fichero_yml> logs
~~~