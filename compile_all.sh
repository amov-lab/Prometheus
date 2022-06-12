#!/bin/bash
# 脚本名称: compile_all.sh
# 脚本描述: 编译Prometheus所有开源模块

# 编译基础模块
catkin_make --source Modules/common --build build/common
# 编译Gazebo仿真模块
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
# 编译控制模块
catkin_make --source Modules/uav_control --build build/uav_control
# 编译demo模块
catkin_make --source Modules/tutorial_demo --build build/tutorial_demo
# 编译规划相关模块
catkin_make --source Modules/simulator_utils --build build/simulator_utils
catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm
catkin_make --source Modules/motion_planning --build build/motion_planning
# 编译目标检测模块
catkin_make --source Modules/object_detection --build build/object_detection
# compile object_detection_yolov5tensorrt
if [ ! -f "Modules/object_detection_yolov5openvino/CMakeLists.txt" ]; then
  # submodule object_detection_yolov5openvino not exist, skip it
  echo -e "\e[32m[INFO] SUBMODULE\e[0m \e[33mobject_detection_yolov5openvino\e[0m \e[32mNOT EXIST, Skip it!\e[0m"
else
  echo -e "\e[32m[INFO] COMPILE \e[33mobject_detection_yolov5openvino\e[0m \e[32m...\e[0m"
  # compile object_detection_landing
  catkin_make --source Modules/object_detection_yolov5openvino --build build/object_detection_yolov5openvino
fi