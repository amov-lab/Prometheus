#!/bin/bash
# 脚本描述：框选跟踪



gnome-terminal -- bash -c "roslaunch prometheus_demo nano_track_P450.launch; exec bash"

echo “prometheus_nano_track  successfully started”
# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore,导致其中一个roslaunch失败,报runid错误
sleep 0.7s  

gnome-terminal -- bash -c "source spirecv-ros/devel/setup.bash;roslaunch spirecv_ros single_object_tracking.launch; exec bash"

echo “spirecv_single_object_tracking  successfully started”

