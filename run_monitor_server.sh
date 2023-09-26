#!/bin/bash

docker run -d --name "robot_monitor_server" \
    --restart "always" \
    --tty \
    --privileged \
    --network=host \
    --volume /var/run/docker.sock:/var/run/docker.sock \
    --volume /run/dbus:/run/dbus \
    --volume /run/systemd:/run/systemd --pid=host \
    --mount type=bind,source="$(pwd)"/calibration_file,target=/calibration_file \
    monitor_server:latest \
    bash -c "cd /robots_monitor_server/ && \
             gunicorn --bind 0.0.0.0:8080 -t 60 main:app"