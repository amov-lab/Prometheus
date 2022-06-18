#!/bin/bash
# 脚本名称: enu_xyz_pos_control
# 脚本描述: 该脚本为本地坐标系下位置控制demo启动脚本,包含PX4 SITL,Gazebo仿真环境,无人机控制节点以及本地坐标系下位置控制节点

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo sitl_outdoor_1uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_outdoor.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_demo enu_xyz_pos_control.launch; exec bash"' \
#--tab -e 'bash -c "sleep 7; rosrun prometheus_demo enu_xyz_pos_control.py; exec bash"' \