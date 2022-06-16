#!/bin/bash
# 脚本描述： SiamRPN框选目标，并跟踪目标(需要GPU,且先配置按照文档配置环境)
# 相关文档： Modules/tutorial_demo/advanced/siamrpn_track/readme.md

# 启动PX4 SITL及Gazebo仿真环境
roslaunch prometheus_demo sitl_outdoor_1uav_yolov5_track.launch & sleep 2s;
# 启动无人机控制 - uav_control_main
roslaunch prometheus_uav_control uav_control_main_outdoor.launch & sleep 2s;
# 启动视觉模块
roslaunch prometheus_demo siamrpn_detection.launch & sleep 2s;
# 启动无人机控制
roslaunch prometheus_demo siamrpn_track.launch & sleep 2s;
# 启动rqt_image_view,用于查看检测
rqt_image_view