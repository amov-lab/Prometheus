# 脚本描述: 
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_p450_2dlidar_swarm.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_uav_control uav_control_main_intdoor_swarm.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_gazebo filter_lidar_swarm.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_gazebo scan_to_octomap_swarm.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch ego_planner sitl_ego_planner_basic_octomap_lidar_swarm.launch; exec bash"' \



