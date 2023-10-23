#!/bin/bash

docker buildx build \
    -f dockerfiles/base/Dockerfile.arm64 \
    -t dkhoanguyen/robotic_base:latest .

