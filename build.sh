#!/bin/bash

docker buildx build \
    -f dockerfiles/base/Dockerfile.arm64 \
    -t robotic_base:latest .

