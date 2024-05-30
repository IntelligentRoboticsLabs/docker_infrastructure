
# 2024 ACM SIGSOFT Summer School on Software Engineering for Robotics
# Installation Script - Practical session in swarm robotics
# Moular design of control software for robot swarms
# David Garzón Ramos <david.garzonramos@bristol.ac.uk>

# Installing dependencies and required packages
sudo apt-get install git cmake libfreeimage-dev libfreeimageplus-dev libqt6opengl6-dev freeglut3-dev libxi-dev libxmu-dev liblua5.2-dev lua5.2 doxygen graphviz graphviz-dev asciidoc npm -y

# Clonning the session repository
cd ~
git clone https://github.com/dagarzonr/sigsoft-swarms.git

# Sourcing environmental variables
cd ~/sigsoft-swarms
source argos3-env.sh

# Cloning, compiling and locally installing the ARGoS3 simulator
cd ~/sigsoft-swarms
mkdir argos3-dist
cd ~/sigsoft-swarms/argos3-installation
git clone https://github.com/ilpincy/argos3.git -b 3.0.0-beta48
cd ~/sigsoft-swarms/argos3-installation/argos3
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$ARGOS_INSTALL_PATH/argos3-dist -DCMAKE_BUILD_TYPE=Release -DARGOS_INSTALL_LDSOCONF=OFF ../src
make -j4
make doc
make install

# Removing the legacy versions of the e-puck library
rm -rf $ARGOS_INSTALL_PATH/argos3-dist/include/argos3/plugins/robots/e-puck
rm -rf $ARGOS_INSTALL_PATH/argos3-dist/lib/argos3/lib*epuck*.so

# Compile and install locally the e-puck libraries v48
cd ~/sigsoft-swarms/argos3-installation/argos3-epuck
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$ARGOS_INSTALL_PATH/argos3-dist -DCMAKE_BUILD_TYPE=Release ../src
make -j4
make install

# Compiling and installing locally the MoCA libraries
cd ~/sigsoft-swarms/argos3-installation/argos3-arena
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$ARGOS_INSTALL_PATH/argos3-dist -DCMAKE_BUILD_TYPE=Release ../src
make -j4
make install

# Compiling and installing locally the loop-functions libraries
cd ~/sigsoft-swarms/argos3-installation/experiments-loop-functions
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$ARGOS_INSTALL_PATH/argos3-dist -DCMAKE_BUILD_TYPE=Release ..
make -j4
make install

# Compiling and installing locally the e-puck DAO libraries
cd ~/sigsoft-swarms/argos3-installation/demiurge-epuck-dao
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$ARGOS_INSTALL_PATH/argos3-dist -DCMAKE_BUILD_TYPE=Release ..
make -j4
make install

# Compiling AutoMoDe TuttiFrutti
cd ~/sigsoft-swarms/argos3-installation/AutoMoDe-tuttifrutti
mkdir build && cd build
cmake ..
make -j4

# Installing AutoMoDe-Editor to graphically design FSM
cd ~/sigsoft-swarms/automode-editor
npm install

# Exiting to the work directory
cd ~/sigsoft-swarms
