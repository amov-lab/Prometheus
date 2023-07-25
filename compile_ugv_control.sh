#!/bin/bash
# 脚本名称: compile_ugv_control.sh
# 脚本描述: 编译Prometheus小车控制模块

# 编译基础模块
catkin_make --source Modules/common --build build/common

# 编译Gazebo仿真模块
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
catkin_make --source Simulator/realsense_gazebo_plugin --build build/realsense_gazebo_plugin
catkin_make --source Simulator/velodyne_gazebo_plugins --build build/velodyne_gazebo_plugins

catkin_make --source Simulator/wheeltec_robot_gazebo/wheeltec_gazebo_control --build build/wheeltec_robot_gazebo/wheeltec_gazebo_control
catkin_make --source Simulator/wheeltec_robot_gazebo/wheeltec_description --build build/wheeltec_robot_gazebo/wheeltec_description
catkin_make --source Simulator/wheeltec_robot_gazebo/wheeltec_gazebo_function --build build/wheeltec_robot_gazebo/wheeltec_gazebo_function


# 编译小车控制模块
catkin_make --source Modules/ugv_control --build build/ugv_control
# 编译demo模块
catkin_make --source Modules/tutorial_demo --build build/tutorial_demo

