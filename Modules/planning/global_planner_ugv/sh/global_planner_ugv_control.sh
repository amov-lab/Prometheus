#!/bin/bash

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_ugv_control ugv_vrpn.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_ugv_control ugv_control_main.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch bluesea2 LDS-50C-3.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch lingao_base lingao_base_driver.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_global_planner_ugv global_planner_ugv_control.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_global_planner_ugv rviz_global_planner_ugv.launch; exec bash"' \
