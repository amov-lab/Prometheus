#!/bin/bash
# 脚本描述： 使用yolov5对目标进行识别，然后点击识别框使用siamrpn进行目标跟踪(需要GPU，且配置按文档配置好环境)
# 相关文档： Modules/tutorial_demo/advanced/yolov5_track/readme.md

# 启动PX4 SITL及Gazebo仿真环境
roslaunch prometheus_demo sitl_outdoor_1uav_yolov5_track.launch & sleep 2s;
# 启动无人机控制 - uav_control_main
roslaunch prometheus_uav_control uav_control_main_outdoor.launch & sleep 2s;
# 启动视觉模块
roslaunch prometheus_demo yolov5_detection.launch & sleep 2s;
python3 yolov5_trt_ros.py  --image_topic /prometheus/sensor/monocular_front/image_raw & sleep 2s;
# 启动无人机控制
roslaunch prometheus_demo yolov5_track.launch & sleep 2s;
# 启动rqt_image_view,用于查看检测
rqt_image_view