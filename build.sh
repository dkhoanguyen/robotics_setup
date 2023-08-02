#!/bin/bash

docker buildx build \
    -f dockerfiles/base/Dockerfile \
    -t robotic_base:latest .

