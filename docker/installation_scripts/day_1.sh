#!/bin/bash

# File: day_1.sh
# Description: Script to install examples and exercises for Day 1 of the Summer School on Software Engineering in Robotics 2025
# Author: Juan Carlos Manzanares Serrano (juancarlos.serrano@urjc.es)
# Date: 05/06/25
# Institution: Universidad Rey Juan Carlos

# variables for the installation
export pkg_dir=~/ros2_ws

# Clone the repository with simulator
cp -r /examples/* $pkg_dir/src/

cd $pkg_dir 

# Compile the code
colcon build --symlink-install

source ~/.bashrc

echo "================================="
echo "  EXAMPLES & EXERCISE INSTALLED  "
echo "================================="

