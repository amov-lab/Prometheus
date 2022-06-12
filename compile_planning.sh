#!/bin/bash
# 脚本名称: compile_planning.sh
# 脚本描述: 编译Prometheus规划模块

# 编译基础模块
catkin_make --source Modules/common --build build/common
# 编译Gazebo仿真模块
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
# 编译控制模块
catkin_make --source Modules/uav_control --build build/uav_control
# 编译规划相关模块
catkin_make --source Modules/simulator_utils --build build/simulator_utils
catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm
catkin_make --source Modules/motion_planning --build build/motion_planning
