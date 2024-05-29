#!/bin/bash

docker run -p 6080:80 --privileged -env="DISPLAY=$DISPLAY" --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="NVIDIA_VISIBLE_DEVICES=all"  --runtime=nvidia  --gpus all --name sigsoft -d sigsoft:v1.0
