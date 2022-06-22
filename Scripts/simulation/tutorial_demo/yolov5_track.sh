#!/bin/bash
# 脚本描述： 使用yolov5对目标进行识别，然后点击识别框使用siamrpn进行目标跟踪(需要GPU，且配置按文档配置好环境)
# 相关文档： Modules/tutorial_demo/advanced/yolov5_track/readme.md

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_demo sitl_outdoor_1uav_yolov5_track.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_outdoor.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_demo yolov5_detection.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; python3 yolov5_trt_ros.py  --image_topic /prometheus/sensor/monocular_front/image_raw; exec bash"' \
--tab -e 'bash -c "sleep 9; roslaunch prometheus_demo yolov5_track.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; rqt_image_view; exec bash"' \