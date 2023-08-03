#!/bin/bash

# Get the number of running containers
running_containers=$(docker ps --format '{{.Names}}')

if [ -n "$running_containers" ]; then
    for container in "$running_containers"
    do
        if [[ "$container" == "rosbridge" || "$container" == "rpi3-wifiap" ]]; then
            continue
        else
            docker stop "$container"
            docker rm "$container"
        fi
    done
fi