# 脚本描述: 

UAV_ID=1

gnome-terminal --window -e 'bash -c "roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$PX4_SIM_HOST_ADDR; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_airsim airsim_scan_to_octomap.launch uav1_id:='$UAV_ID' uav1_init_x:=-0.0 uav1_init_y:=-0.0; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch prometheus_airsim airsim_sitl_ego_scan_octomap.launch uav_id:='$UAV_ID'; exec bash"' \

