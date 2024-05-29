#!/bin/bash

# variables for the installation
export pkg_dir=/home/ubuntu/ros2_ws

# Update the package list
sudo apt-get update
sudo apt-get upgrade -y

# Clone the repository 
git clone -b humble-devel https://github.com/fmrico/book_ros2.git

# Change directory to the cloned repository
cd $pkg_dir/src/book_ros2

for i in br2_*
do
  touch $i/COLCON_IGNORE
done

rm br2_bt_bumpgo/COLCON_IGNORE

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
