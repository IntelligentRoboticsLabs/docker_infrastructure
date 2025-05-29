#!/bin/bash

# Check if there are arguments
if [ "$#" -eq 0 ]; then
    # No arguments passed, proceed with GPU check
    if lspci | grep -i nvidia; then
        echo "Running Docker container with NVIDIA GPU support."
        docker run -p 6080:80 --privileged --env="DISPLAY=$DISPLAY" --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="NVIDIA_VISIBLE_DEVICES=all" --runtime=nvidia --gpus all --name school -d jmguerreroh/school:ubuntu24
    else
        echo "Running Docker container without NVIDIA GPU support."
        docker run -p 6080:80 --privileged --name school -d jmguerreroh/school:ubuntu24
    fi
elif [ "$1" == "gpu" ]; then
    # Argument 'gpu' passed, force GPU support 
    echo "Running Docker container with NVIDIA GPU support."
    docker run -p 6080:80 --privileged --env="DISPLAY=$DISPLAY" --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="NVIDIA_VISIBLE_DEVICES=all" --runtime=nvidia --gpus all --name school -d jmguerreroh/school:ubuntu24
   
elif [ "$1" == "no-gpu" ]; then
    # Argument 'no-gpu' passed, skip GPU check
    echo "Running Docker container without NVIDIA GPU support."
    docker run -p 6080:80 --privileged --name school -d jmguerreroh/school:ubuntu24
    
else
    echo "Invalid argument. Use 'gpu' to run with NVIDIA GPU support or 'no-gpu' to run without NVIDIA GPU support."
    exit 1
fi
