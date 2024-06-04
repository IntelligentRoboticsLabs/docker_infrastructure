#!/bin/bash

# variables for the installation
export pkg_dir=/home/ubuntu/ros2_ws

# Update the package list
sudo apt-get update
sudo apt-get upgrade -y

# Clone the repository 
cd $pkg_dir/src
git clone https://github.com/SIGSOFT-Summer-Winter-School/Bump-and-Go-with-Behavior-Trees.git
git clone https://github.com/BehaviorTree/Groot.git

# Change directory to the cloned repository
cd $pkg_dir/src/Bump-and-Go-with-Behavior-Tree

cd $pkg_dir 
rosdep install --from-paths src --ignore-src -r -y

# Compile the code
colcon build --symlink-install

# Run any additional commands or scripts as needed
echo "source $pkg_dir/install/setup.bash" >> /home/ubuntu/.bashrc
source /home/ubuntu/.bashrc

echo "================================"
echo "      DAY 1 INSTALLED       "
echo "================================"
