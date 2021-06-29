#!/bin/bash
# source /home/dji/Kongdijiqun/devel/setup.bash

# 启动gazebo,rviz
roslaunch prometheus_gazebo sitl_ego_map4.launch & sleep 10s;
# PX4 sitl
roslaunch prometheus_gazebo sitl_ego_px4.launch & sleep 5s;
# ego
roslaunch prometheus_gazebo sitl_ego_planner.launch & sleep 3s;
# case3 station
roslaunch prometheus_gazebo sitl_ego_station.launch


