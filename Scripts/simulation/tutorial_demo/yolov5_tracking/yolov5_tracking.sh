#!/bin/bash
# 脚本描述：点击二维码无人机跟踪

# 该启动命令为配合SpireCV-x86推流使用，所以需要安装SpireCV，如果没有安装将导致启动失败从而导致推流失败
gnome-terminal -- bash -c "~/SpireCV/ZLM/startMediaServer.sh; exec bash"

sleep 1.5s

gnome-terminal -- bash -c "roslaunch prometheus_demo yolov5_tracking.launch; exec bash"

echo “prometheus_yolov5_tracking  successfully started”
# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore,导致其中一个roslaunch失败,报runid错误
sleep 0.7s  

gnome-terminal -- bash -c "roslaunch spirecv_ros car_detection_with_d435i.launch; exec bash"

echo “spirecv_object_detection  successfully started”

