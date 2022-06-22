#!/bin/bash
# 脚本描述：二维码识别降落
# 相关文档：Modules/tutorial_demo/advanced/autonomous_landing/readme.md

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_demo sitl_outdoor_1uav_autonomous_landing.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_outdoor.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_demo landpad_datection.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch prometheus_demo autonomous_landing.launch; exec bash"' \
--tab -e 'bash -c "sleep 9; rqt_image_view; exec bash"' \