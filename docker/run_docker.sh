#!/bin/bash

if lspci | grep -i nvidia; then
    docker run -p 6080:80 --privileged --env="DISPLAY=$DISPLAY" --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="NVIDIA_VISIBLE_DEVICES=all" --runtime=nvidia --gpus all --name school -d jmguerreroh/school:ubuntu22
    echo "Running Docker container with NVIDIA GPU support."
else
    docker run -p 6080:80 --privileged --name school -d jmguerreroh/school:ubuntu22
    echo "Running Docker container without NVIDIA GPU support."
fi