#!/bin/bash

# Get the number of running containers
running_containers=$(docker ps --format '{{.ID}}')

if [ -n "$running_containers" ]; then
    echo "There are running containers."
    # Force remove wifi ap first to prevent shutting dow
    docker rm -f rpi3-wifiap
    # Stop all running containers
    docker stop $(docker ps -q)

    # Remove all stopped containers
    docker rm $(docker ps -a -q)
fi