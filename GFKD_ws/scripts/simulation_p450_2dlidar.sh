# 脚本描述: 国防科大单机仿真脚本

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch prometheus_gfkd gazebo_world.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gfkd px4_sitl_p450_2dlidar.launch uav1_init_x:=-0.0 uav1_init_y:=-0.0; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_uav_control uav_control_main_outdoor.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gfkd scan_to_octomap.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gfkd ego_planner.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gfkd rviz.launch; exec bash"' \
