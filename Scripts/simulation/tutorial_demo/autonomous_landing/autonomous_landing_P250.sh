#!/bin/bash
# 脚本描述：二维码识别降落



gnome-terminal -- bash -c "roslaunch prometheus_demo autonomous_landing_all_P250.launch; exec bash"

echo “prometheus_autonomous_landing  successfully started”
# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore,导致其中一个roslaunch失败,报runid错误
sleep 0.7s  

gnome-terminal -- bash -c "roslaunch spirecv_ros aruco_detection_with_rostopic.launch; exec bash"

echo “spirecv_aruco_detection  successfully started”

