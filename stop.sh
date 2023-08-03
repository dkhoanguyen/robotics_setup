#!/bin/bash

# Add the container names you want to keep to this array
container_names_to_keep=("rosbridge" "rpi3-wifiap" )

# Get the IDs of all running containers (excluding the ones to keep)
container_ids_to_stop=$(docker ps -q --filter "name!=${container_names_to_keep[*]}")

# Stop the containers (if any)
if [[ -n "$container_ids_to_stop" ]]; then
    docker stop $container_ids_to_stop
    docker rm $container_ids_to_stop
    echo "Stopped all containers except: ${container_names_to_keep[*]}."
else
    echo "No containers to stop except: ${container_names_to_keep[*]}."
fi