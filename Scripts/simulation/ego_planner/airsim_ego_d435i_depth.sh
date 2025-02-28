# 脚本描述: 

UAV_ID=1

gnome-terminal --window -e 'bash -c "roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$PX4_SIM_HOST_ADDR; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_airsim sitl_ego_depth_airsim.launch uav1_id:='$UAV_ID' uav1_init_x:=-0.0 uav1_init_y:=-0.0; exec bash"' \
