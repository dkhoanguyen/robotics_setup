#!/bin/bash

docker run -it 
    --name="ur3e_controller" \
    --privileged="true" \
    --restart="always" \
    --network="host" \
    robotic_base:latest bash
