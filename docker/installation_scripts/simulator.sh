#!/bin/bash

# File: simulator.sh
# Description: Script to install Tiago simulator for Summer School on Software Engineering in Robotics 2024
# Author: Jose Miguel Guerrero (josemiguel.guerrero@urjc.es)
# Date: 30/05/24
# Institution: Universidad Rey Juan Carlos

# variables for the installation
export pkg_dir=/home/ubuntu/ros2_ws

# Update the package list
sudo apt-get update
sudo apt-get upgrade -y

# Clone the repository with simulator
git clone -b jazzy https://github.com/Juancams/mirte-ros-packages.git $pkg_dir/src/mirte-ros-packages
git clone -b jazzy https://github.com/Juancams/mirte-gazebo.git $pkg_dir/src/mirte-gazebo
git clone -b ros2-jazzy https://github.com/Juancams/aws-robomaker-small-warehouse-world.git $pkg_dir/src/aws-robomaker-small-warehouse-world

touch $pkg_dir/src/mirte-ros-packages/mirte_telemetrix_cpp/COLCON_IGNORE

cd $pkg_dir 
rosdep install --from-paths src --ignore-src -r -y

# Compile the code
colcon build --symlink-install

# Run any additional commands or scripts as needed
echo "source $pkg_dir/install/setup.bash" >> /home/ubuntu/.bashrc
source /home/ubuntu/.bashrc

echo "================================"
echo "      SIMULATOR INSTALLED       "
echo "================================"

