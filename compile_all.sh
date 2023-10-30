#!/bin/bash
###
 # @Author: Yuhua.Qi fatmoonqyp@126.com
 # @Date: 2023-10-17 20:54:52
 # @LastEditors: Yuhua.Qi fatmoonqyp@126.com
 # @LastEditTime: 2023-10-30 13:50:15
 # @FilePath: /Prometheus/compile_all.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 
# 脚本名称: compile_all.sh
# 脚本描述: 编译Prometheus所有开源模块

# 编译基础模块
catkin_make --source Modules/common --build build/common

# 编译通信模块
catkin_make --source Modules/communication --build build/communication

# 编译Gazebo仿真模块
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
catkin_make --source Simulator/realsense_gazebo_plugin --build build/realsense_gazebo_plugin
catkin_make --source Simulator/velodyne_gazebo_plugins --build build/velodyne_gazebo_plugins

# 编译控制模块
catkin_make --source Modules/uav_control --build build/uav_control
# 编译demo模块
catkin_make --source Modules/tutorial_demo --build build/tutorial_demo
# 编译规划相关模块
catkin_make --source Modules/simulator_utils --build build/simulator_utils
catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm
catkin_make --source Modules/motion_planning --build build/motion_planning
# 编译目标检测模块
# catkin_make --source Modules/object_detection --build build/object_detection
