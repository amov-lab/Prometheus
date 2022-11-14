#!/bin/bash
# 脚本名称: takeoff_land
# 脚本描述: 该脚本为起飞&降落控制demo启动脚本,包含PX4 SITL,Gazebo仿真环境,无人机控制节点以及起飞&降落控制节点

gnome-terminal --window -e 'bash -c "sleep 2; roslaunch prometheus_communication_bridge simulation_bridge.launch; exec bash"' \
