# 脚本描述: 

UAV_ID=1

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_outdoor_1uav_P600_mid360.launch uav1_id:='$UAV_ID' uav1_init_x:=-0.0 uav1_init_y:=-0.0; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch prometheus_uav_control uav_control_main_outdoor.launch uav_id:='$UAV_ID'; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch fast_lio mapping_mid360_gazebo.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch ego_planner sitl_ego_planner_basic_octomap_mid360.launch uav_id:='$UAV_ID'; exec bash"' \


