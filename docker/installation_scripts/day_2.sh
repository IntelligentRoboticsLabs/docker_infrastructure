#!/bin/bash

# File: day_2.sh
# Description: Script to install rebet for the Summerschool on Software Engineering in Robotics 2024
# Author: Elvin Alberts (e.g.alberts@vu.nl)
# Date: 30/05/24
# Institution: Vrije Universiteit Amsterdam

# variables for the installation
export pkg_dir=/home/ubuntu/rebet_ws

mkdir -p $pkg_dir/src/

# Update the package list
sudo apt-get update
sudo apt-get upgrade -y

pip install ultralytics
pip install masced_bandits

cd $pkg_dir/src/

wget -O AAL_ROS2.zip https://anonymous.4open.science/api/repo/AAL-ROS2-0508/zip
wget -O REBET_DEMO.zip https://anonymous.4open.science/api/repo/ReBeT-DEMO/zip

unzip AAL_ROS2.zip -d AAL_ROS2
unzip REBET_DEMO.zip -d REBET_DEMO

rm AAL_ROS2.zip
rm REBET_DEMO.zip

git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git

wget -O BTCPP.zip https://github.com/BehaviorTree/BehaviorTree.CPP/archive/refs/tags/4.6.1.zip
unzip BTCPP.zip
rm BTCPP.zip



cd $pkg_dir 
rosdep install --from-paths src --ignore-src -r -y

# Compile the code
colcon build --packages-skip rebet_demo
source install/setup.bash
colcon build --packages-select rebet_demo

# Run any additional commands or scripts as needed
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/rebet_ws/install/rebet_demo/share/rebet_demo/models/' >> ~/.bashrc
wget -O /home/ubuntu/ros2_ws/src/ThirdParty/aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world https://raw.githubusercontent.com/EGAlberts/aws-robomaker-small-warehouse-world/ros2/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world
echo "================================"
echo "        DAY 2 INSTALLED         "
echo "================================"

echo "source $pkg_dir/install/setup.bash" >> /home/ubuntu/.bashrc
source /home/ubuntu/.bashrc