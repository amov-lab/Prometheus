# 脚本描述: 

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_p450_d435i.launch uav1_init_x:=-0.0 uav1_init_y:=-0.0; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch prometheus_uav_control uav_control_main_outdoor.launch; exec bash"' \
# --tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo depth_to_octomap.launch; exec bash"' \
# --tab -e 'bash -c "sleep 6; roslaunch ego_planner sitl_ego_planner_basic_octomap.launch; exec bash"' \

