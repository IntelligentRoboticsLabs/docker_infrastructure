#!/bin/bash

# File: day_4_verification.sh
# Description: Script to initialize the statistical model checking toolchain for Summerschool on Software Engineering in Robotics 2024
# Author: Matteo Palmas (matteo.palmas@de.bosch.com), Michaela Klauck (michaela.klauck@de.bosch.com)
# Date: 29/05/24
# Institution: Robert Bosch GmbH, Bosch Research 

sudo apt-get update -y
sudo apt-get upgrade -y

# Define the base STORM directory (Directory where all smc-related sources are stored)
export STORM_BASE_DIR=~/Desktop/STORM_DIR
# Export the STORM directory name (Storm directory is where all the source code is stored)
export STORM_DIR=$STORM_BASE_DIR/storm
# Export the directory for the storm smc source code
export STORM_SMC_DIR=$STORM_BASE_DIR/smc_storm
# Export the directory for the python jani source code
export MC_JANI=$STORM_BASE_DIR/mc-toolchain-jani


# Set the export in the bashrc file to have them run when a new shell is opened
echo "export STORM_BASE_DIR="$STORM_BASE_DIR >> ~/.bashrc
echo "export STORM_DIR="$STORM_DIR >> ~/.bashrc
echo "export STORM_SMC_DIR="$STORM_SMC_DIR >> ~/.bashrc
echo "export MC_JANI="$MC_JANI >> ~/.bashrc

# Create the base STORM directory
mkdir $STORM_BASE_DIR

# Install STORM dependencies
sudo apt-get install build-essential git cmake libboost-all-dev libcln-dev libgmp-dev libginac-dev automake libglpk-dev libhwloc-dev libz3-dev libxerces-c-dev libeigen3-dev -y

# Go to the STORM base directory
cd $STORM_BASE_DIR
# Clone the updated STORM repository
git clone -b support-for-sin-cos-constants https://github.com/boschresearch/storm.git

# Build the STORM source code
cd $STORM_DIR
mkdir build
cd build
# NOTE: THIS TAKES A WHILE...
cmake -DSTORM_USE_SPOT_SHIPPED=ON ..
make

export PATH=$PATH:$STORM_DIR/build
echo "export PATH=$PATH:$STORM_DIR/build/bin" >> ~/.bashrc
echo "alias storm=$STORM_DIR/build/bin/storm" >> ~/.bashrc
# Go to STORM base directory
cd $STORM_BASE_DIR
# Clone the SMC STORM repository
git clone  https://github.com/convince-project/smc_storm.git 

# Compile the source code for SMC STORM
cd $STORM_BASE_DIR/smc_storm
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4

export PATH=$PATH:$STORM_SMC_DIR/build
echo "export PATH=$PATH:$STORM_SMC_DIR/build" >> ~/.bashrc

# Clone Model Checking Toolchain for JANI
cd $STORM_BASE_DIR

# Clone the source code from the repository
git clone https://github.com/convince-project/mc-toolchain-jani.git

cd $MC_JANI

# Install the python codes with pip
python3 -m pip install mc_toolchain_jani_common/
python3 -m pip install jani_generator/
python3 -m pip install scxml_converter/

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc


