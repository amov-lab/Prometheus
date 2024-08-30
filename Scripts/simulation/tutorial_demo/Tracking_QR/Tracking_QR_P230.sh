#!/bin/bash
# 脚本描述：点击二维码无人机跟踪

# 该启动命令为配合SpireCV-x86推流使用，所以需要安装SpireCV，如果没有安装将导致启动失败从而导致推流失败
gnome-terminal -- bash -c "~/SpireCV/ZLM/startMediaServer.sh; exec bash"

sleep 1.5s

# 获取 prometheus_gazebo 的路径
PROMETHEUS_GAZEBO_PATH=$(rospack find prometheus_gazebo)
# 二维码 world 文件路径
WORLD_FILE="$PROMETHEUS_GAZEBO_PATH/gazebo_worlds/detection_worlds/Tracking_QR/Tracking_QR.world"

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_indoor_1uav_P230.launch vehicle:='p230_D435i' d435i_enable:=true world:='$WORLD_FILE'; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_indoor.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch prometheus_demo aruco_tracking_control.launch; exec bash"' \

echo “prometheus_Tracking_QR  successfully started”
# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore,导致其中一个roslaunch失败,报runid错误
sleep 0.7s  

gnome-terminal -- bash -c "roslaunch spirecv_ros aruco_detection_with_d435i.launch; exec bash"

echo “spirecv_aruco_detection  successfully started”

