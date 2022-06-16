#!/bin/bash
# 脚本描述：二维码识别降落
# 相关文档：Modules/tutorial_demo/advanced/autonomous_landing/readme.md

# 启动PX4 SITL及Gazebo仿真环境
roslaunch prometheus_demo sitl_outdoor_1uav_autonomous_landing.launch & sleep 2s;
# 启动无人机控制 - uav_control_main
roslaunch prometheus_uav_control uav_control_main_outdoor.launch & sleep 2s;
# 启动视觉模块
roslaunch prometheus_demo landpad_datection.launch & sleep 2s;
# 启动降落控制程序
roslaunch prometheus_demo autonomous_landing.launch & sleep 2s;
# 启动rqt_image_view,用于查看检测
rqt_image_view