#!/bin/bash

# File: simulator.sh
# Description: Script to install Mirte simulator and exercises for Summer School on Software Engineering in Robotics 2025
# Author: Jose Miguel Guerrero (josemiguel.guerrero@urjc.es)
# Date: 23/06/25
# Institution: Universidad Rey Juan Carlos

# variables for the installation
export pkg_dir=~/ros2_ws

# Update the package list
sudo apt update
sudo apt full-upgrade -y

# Clone the repository with simulator
git clone -b jazzy https://github.com/Juancams/mirte-ros-packages.git $pkg_dir/src/mirte-ros-packages
git clone -b harmonic https://github.com/Juancams/mirte-gazebo.git $pkg_dir/src/mirte-gazebo
git clone -b ros2-jazzy https://github.com/Juancams/aws-robomaker-small-warehouse-world.git $pkg_dir/src/aws-robomaker-small-warehouse-world

touch $pkg_dir/src/mirte-ros-packages/mirte_telemetrix_cpp/COLCON_IGNORE

# Clone the example
cp -r ../examples/* $pkg_dir/src/

cd $pkg_dir 
rosdep install --from-paths src --ignore-src -r -y

# Compile the code
colcon build --symlink-install

# Run any additional commands or scripts as needed
echo "source $pkg_dir/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "====================================================="
echo "      SIMULATOR, EXAMPLES & EXERCISE INSTALLED       "
echo "====================================================="
