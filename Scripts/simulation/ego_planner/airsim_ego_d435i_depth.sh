# 脚本描述: 

UAV_ID=1

gnome-terminal --window -e 'bash -c " source /opt/ros/noetic/setup.bash;source ~/Prometheus/devel/setup.bash;source /home/amov/prometheus_mavros/devel/setup.bash;source ~/prometheus_px4/Tools/setup_gazebo.bash ~/prometheus_px4 ~/prometheus_px4/build/amovlab_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4/Tools/sitl_gazebo ;roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$PX4_SIM_HOST_ADDR;exit; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_airsim sitl_ego_depth_airsim.launch uav1_id:='$UAV_ID' uav1_init_x:=-0.0 uav1_init_y:=-0.0;exit; exec bash"' \
