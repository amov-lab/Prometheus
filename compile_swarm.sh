#!/bin/bash
# 脚本名称: compile_swarm.sh
# 脚本描述: 编译Prometheus swarm_control 模块

# 编译基础模块
catkin_make --source Modules/common --build build/common
# 编译通信模块
catkin_make --source Modules/communication --build build/communication
# 编译Gazebo仿真模块
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
# 编译控制模块
catkin_make --source Modules/uav_control --build build/uav_control

# 编译小车控制模块
catkin_make --source Modules/ugv_control --build build/ugv_control
# 编译swarm_control模块
catkin_make --source Modules/swarm_control --build build/swarm_control

# 编译swarm_formation模块
catkin_make --source Modules/swarm_formation --build build/swarm_formation

