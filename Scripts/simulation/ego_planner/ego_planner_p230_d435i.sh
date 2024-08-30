# 脚本描述: 

UAV_ID=1

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_p230_d435i.launch uav1_id:='$UAV_ID'; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch prometheus_uav_control uav_control_main_indoor.launch uav_id:='$UAV_ID'; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo depth_to_octomap.launch uav_id:='$UAV_ID'; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch ego_planner sitl_ego_planner_basic_octomap.launch uav_id:='$UAV_ID'; exec bash"' \

