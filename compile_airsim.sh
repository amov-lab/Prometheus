#!/bin/bash
# 脚本名称: compile_airsim.sh
# 脚本描述: 编译Prometheus airsim模块

# 编译基础模块
catkin_make --source Modules/common --build build/common
# 编译控制模块
catkin_make --source Modules/uav_control --build build/uav_control
# 编译通信模块
catkin_make --source Modules/communication --build build/communication
# 编译AirSim仿真模块
catkin_make --source Simulator/airsim_simulator --build build/prometheus_airsim
catkin_make --source Simulator/airsim_simulator/airsim_ros/ros --build build/prometheus_airsim_ros


