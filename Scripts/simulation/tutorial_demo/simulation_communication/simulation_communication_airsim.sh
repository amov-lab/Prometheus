#!/bin/bash
# 脚本名称: takeoff_land
# 脚本描述: 该脚本为起飞&降落控制demo启动脚本,包含PX4 SITL,Gazebo仿真环境,无人机控制节点以及起飞&降落控制节点

gnome-terminal --window -e 'bash -c "export PX4_SIM_HOST_ADDR=172.30.224.1; source /opt/ros/noetic/setup.bash;source ~/Prometheus/devel/setup.bash;source /home/amov/prometheus_mavros/devel/setup.bash;source ~/prometheus_px4/Tools/setup_gazebo.bash ~/prometheus_px4 ~/prometheus_px4/build/amovlab_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4/Tools/sitl_gazebo ; roslaunch prometheus_communication_bridge simulation_bridge.launch; exec bash"' \
--tab -e 'bash -c "source /opt/ros/noetic/setup.bash;sleep 4; roslaunch rosbridge_server rosbridge_websocket.launch; exec bash"' \
