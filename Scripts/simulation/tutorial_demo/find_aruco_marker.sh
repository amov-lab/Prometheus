#!/bin/bash
# 脚本描述： 无人机绕圆飞行，找相应的二维码，并前往相应二维码进行降落
# 相关文档： Modules/tutorial_demo/advanced/find_aruco_marker/readme.md

# 启动PX4 SITL及Gazebo仿真环境
roslaunch prometheus_demo sitl_outdoor_1uav_find_aruco_marker.launch & sleep 2s;
# 启动无人机控制 - uav_control_main
roslaunch prometheus_uav_control uav_control_main_outdoor.launch & sleep 2s;
# 启动视觉模块
roslaunch prometheus_demo aruco_datection.launch & sleep 2s;
# 启动无人机控制
roslaunch prometheus_demo find_aruco_marker.launch & sleep 2s;
# 启动rqt_image_view,用于查看检测
rqt_image_view

