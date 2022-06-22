#!/bin/bash
# 脚本描述： SiamRPN框选目标，并跟踪目标(需要GPU,且先配置按照文档配置环境)
# 相关文档： Modules/tutorial_demo/advanced/siamrpn_track/readme.md

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_demo sitl_outdoor_1uav_yolov5_track.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_outdoor.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_demo siamrpn_detection.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch prometheus_demo siamrpn_track.launch; exec bash"' \
--tab -e 'bash -c "sleep 9; rqt_image_view; exec bash"' \