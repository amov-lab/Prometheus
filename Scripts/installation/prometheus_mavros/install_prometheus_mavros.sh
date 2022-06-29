#!/bin/bash
SHELL_FOLDER=$(dirname $(readlink -f "$0"))
sudo apt update &&\
sudo apt-get install python-catkin-tools python-rosinstall-generator -y &&\

mkdir -p ~/prometheus_mavros/src &&\
cd ~/prometheus_mavros &&\
catkin init &&\
wstool init src

wstool merge -t src $SHELL_FOLDER/mavros.rosinstall &&\
wstool update -t src -j$(nproc) &&\
rosdep install --from-paths src --ignore-src -y &&\

sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh &&\

catkin build &&\
source $HOME/prometheus_mavros/devel/setup.bash &&\

#source_command="export ROS_PACKAGE_PATH=${HOME}/prometheus_mavros:$ROS_PACKAGE_PATH"
source_command="source $HOME/prometheus_mavros/devel/setup.bash"
if [ `grep -c "$source_command" ~/.bashrc` -eq '0' ];then
    echo $source_command >> ~/.bashrc
fi
