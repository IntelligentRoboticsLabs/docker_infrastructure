#!/bin/bash

docker run -p 6080:80 --privileged -env="DISPLAY=$DISPLAY" --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="NVIDIA_VISIBLE_DEVICES=all"  --runtime=nvidia  --gpus all --name school -d school:v1.0
