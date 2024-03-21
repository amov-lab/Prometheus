#!/bin/bash
# 脚本描述：点击二维码无人机跟踪



gnome-terminal -- bash -c "roslaunch prometheus_demo Tracking_QR_P250.launch; exec bash"

echo “prometheus_Tracking_QR  successfully started”
# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore,导致其中一个roslaunch失败,报runid错误
sleep 0.7s  

gnome-terminal -- bash -c "roslaunch spirecv_ros aruco_detection_with_single_object_tracking.launch; exec bash"

echo “spirecv_aruco_detection  successfully started”

