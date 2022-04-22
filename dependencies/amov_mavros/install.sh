#!/bin/bash
SHELL_FOLDER=$(dirname $(readlink -f "$0"))
sudo apt update
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
# For Noetic use that:
# sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y

# 1. Create the workspace: unneeded if you already has workspace
mkdir -p ~/prometheus_mavros/src
cd ~/prometheus_mavros
catkin init
wstool init src

# 4. Create workspace & deps
wstool merge -t src $SHELL_FOLDER/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y

# 5. Install GeographicLib datasets:
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# 6. Build source
catkin build

# 7. Make sure that you use setup.bash or setup.zsh from workspace.
#    Else rosrun can't find nodes from this workspace.
source devel/setup.bash

# 8. 加入.bashrc
echo "source ${SHELL_FOLDER}/devel/setup.bash" >> ~/.bashrc




