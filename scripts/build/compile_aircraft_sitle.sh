#!/bin/bash
# 编译基础模块
catkin_make --source Modules/core/common --build build/common
# 编译Gazebo仿真模块
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
# 编译控制模块
catkin_make --source Modules/control/uav_control --build build/uav_control
# 编译demo模块
catkin_make --source Modules/perception/simulator_utils --build build/simulator_utils
# 编译目标检测模块 (object_detection 目录不在仓库中，按需取消注释)
# catkin_make --source Modules/perception/object_detection --build build/object_detection
# aircraft_sitle
catkin_make --source Modules/demo/future_aircraft --build build/future_aircraft
