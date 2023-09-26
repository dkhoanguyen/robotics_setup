#!/bin/bash

docker run -d --name "robot_monitor_server" \
    --restart "always" \
    --tty \
    --privileged \
    --network=host \
    --volume /var/run/docker.sock:/var/run/docker.sock \
    monitor_server:latest \
    bash -c "cd /usr/src/app/src/ && \
             gunicorn --bind 0.0.0.0:8080 main:app"