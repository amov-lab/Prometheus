#!/bin/bash
# 脚本名称: compile_spirecv.sh
# 脚本描述: 编译Prometheus目标检测模块

# 编译基础模块
catkin_make --source Modules/common --build build/common
# 编译Gazebo仿真模块
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
# 编译控制模块
catkin_make --source Modules/uav_control --build build/uav_control
# 编译demo模块
catkin_make --source Modules/tutorial_demo --build build/tutorial_demo
# 编译目标检测模块
release_num=$(lsb_release -r --short)
echo $release_num
if [ $release_num == "18.04" ]
then
  catkin_make --source Modules/spirecv-ros/cv_bridge_1804 --build build/cv_bridge
else
  catkin_make --source Modules/spirecv-ros/cv_bridge_2004 --build build/cv_bridge
fi
catkin_make --source Modules/spirecv-ros/sv-msgs --build build/msgs
catkin_make --source Modules/spirecv-ros/sv-srvs --build build/srvs
catkin_make --source Modules/spirecv-ros/sv-rosapp --build build/rosapp

