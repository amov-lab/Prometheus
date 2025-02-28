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

# 编译demo模块
catkin_make --source Modules/tutorial_demo --build build/tutorial_demo

# 编译规划相关模块
catkin_make --source Modules/simulator_utils --build build/simulator_utils
catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm
catkin_make --source Modules/motion_planning --build build/motion_planning
