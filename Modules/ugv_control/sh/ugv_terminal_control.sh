#!/bin/bash

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_ugv_control ugv1_vrpn.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_ugv_control ugv_control_main.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_ugv_control ugv_terminal_control.launch; exec bash"' \
