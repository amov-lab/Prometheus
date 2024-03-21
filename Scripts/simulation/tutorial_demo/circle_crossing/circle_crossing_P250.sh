#!/bin/bash
# 脚本描述： 无人机圆框检测，并穿越圆框

gnome-terminal -- bash -c "roslaunch prometheus_demo circle_crossing_P250.launch; exec bash"

echo “prometheus_ circle_crossing  successfully started”
# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore,导致其中一个roslaunch失败,报runid错误
sleep 0.7s  

gnome-terminal -- bash -c "roslaunch spirecv_ros ellipse_detection.launch ; exec bash"

echo “spirecv_ circle_crossing  successfully started”

