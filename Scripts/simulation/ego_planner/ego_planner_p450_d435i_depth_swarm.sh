# 脚本描述: 
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_p450_d435i_swarm.launch ; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch prometheus_uav_control uav_control_main_intdoor_swarm.launch ; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch ego_planner sitl_ego_planner_basic_octomap_d435i_swarm_depth.launch; exec bash"' \

