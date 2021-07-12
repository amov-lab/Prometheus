#!/bin/bash
# source /home/dji/Kongdijiqun/devel/setup.bash

# 启动gazebo,rviz
roslaunch prometheus_gazebo sitl_ego_map4.launch & sleep 8s;
# PX4 sitl
roslaunch prometheus_gazebo sitl_ego_px4_uav1.launch & sleep 4s;
# PX4 sitl
# roslaunch prometheus_gazebo sitl_ego_px4_uav2.launch & sleep 4s;
# ego
roslaunch prometheus_gazebo sitl_ego_planner_uav1.launch & sleep 2s;
# ego
# roslaunch prometheus_gazebo sitl_ego_planner_uav2.launch & sleep 2s;
# station
roslaunch prometheus_gazebo sitl_ego_station.launch


