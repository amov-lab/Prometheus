# 脚本名称: ego_planner_1uav_gfkd
# 脚本描述: 国防科大单机ego_planner算法测试 - scan

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo gfkd_rviz.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_gazebo sitl_indoor_1uav_scan.launch uav1_init_x:=-6.0 uav1_init_y:=-7.0; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_uav_control uav_control_main_indoor.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch ego_planner sitl_ego_planner_basic_octomap.launch; exec bash"' \