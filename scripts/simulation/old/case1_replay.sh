#!/bin/bash
# source /home/dji/Kongdijiqun/devel/setup.bash
# 启动脚本：case2_simulation_40uav_20ugv

# 启动Gazebo仿真环境 - Gazebo、rviz、fake_odom
roslaunch prometheus_gazebo sitl_cxy_case1_gazebo_fake_odom.launch swarm_num_uav:=40 fake_odom:=false & sleep 5s;
# 启动地面站 - cxy_ground_station_sub、formation_control
# roslaunch prometheus_gazebo sitl_cxy_case1_station_ego.launch swarm_num_uav:=40 formation_size:=1.0
# ROS bag回放
rosbag play ~/rosbag_cxy/case1.bag -l