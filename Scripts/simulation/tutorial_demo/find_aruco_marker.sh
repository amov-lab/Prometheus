#!/bin/bash
# 脚本描述： 无人机绕圆飞行，找相应的二维码，并前往相应二维码进行降落
# 相关文档： Modules/tutorial_demo/advanced/find_aruco_marker/readme.md

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_demo sitl_outdoor_1uav_find_aruco_marker.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_outdoor.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_demo aruco_datection.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch prometheus_demo find_aruco_marker.launch; exec bash"' \
--tab -e 'bash -c "sleep 9; rqt_image_view; exec bash"' \