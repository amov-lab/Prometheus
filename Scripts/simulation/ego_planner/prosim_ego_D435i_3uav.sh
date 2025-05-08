gnome-terminal --window -e 'bash -c "roscore; exit; exec bash"' \
--tab -e 'bash -c "source ~/prometheus_px4/Tools/setup_gazebo.bash ~/prometheus_px4 ~/prometheus_px4/build/amovlab_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4/Tools/sitl_gazebo;sleep 3; roslaunch prometheus_airsim sitl_px4_outdoor_airsim_3uav.launch; exit; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch prometheus_uav_control uav_control_main_outdoor_3uav.launch; exit; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$PX4_SIM_HOST_ADDR; exit; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_airsim airsim_sitl_ego_depth_3uav.launch; exit; exec bash"' \




