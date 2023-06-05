#!/bin/bash

# 编译基础模块
catkin_make --source Modules/common --build build/common
# 编译Gazebo仿真模块
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
# 编译控制模块
catkin_make --source Modules/uav_control_fmt --build build/uav_control_fmt
