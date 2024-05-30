#!/bin/bash

# Check if an argument has been passed
if [ -z "$1" ]; then
  echo "Please provide an argument: 'ubuntu22' or 'ubuntu20'."
  exit 1
fi
if [ "$1" == "ubuntu22" ] || [ "$1" == "ubuntu20" ]; then
    if lspci | grep -i nvidia; then
        docker run -p 6080:80 --privileged --env="DISPLAY=$DISPLAY" --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="NVIDIA_VISIBLE_DEVICES=all" --runtime=nvidia --gpus all --name school_"$1" -d jmguerreroh/school:"$1"
        echo "Running Docker container with NVIDIA GPU support."
    else
        docker run -p 6080:80 --privileged --name school_"$1" -d jmguerreroh/school:"$1"
        echo "Running Docker container without NVIDIA GPU support."
    fi
else
    echo "Invalid argument. Please provide 'ubuntu22' or 'ubuntu20'."
    exit 1
fi