# 脚本描述: 
UAV_ID=1
gnome-terminal --window -e 'bash -c "source /opt/ros/noetic/setup.bash;source ~/Prometheus/devel/setup.bash;source /home/amov/prometheus_mavros/devel/setup.bash;source ~/prometheus_px4/Tools/setup_gazebo.bash ~/prometheus_px4 ~/prometheus_px4/build/amovlab_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4/Tools/sitl_gazebo ;roslaunch prometheus_airsim sitl_px4_indoor_airsim.launch; exit; exec bash"' \
--tab -e 'bash -c "sleep 3;roslaunch prometheus_airsim uav_control_main_indoor_mid360_airsim.launch; exit; exec bash"' \
--tab -e 'bash -c "sleep 3;roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$PX4_SIM_HOST_ADDR; exit; exec bash"' \
--tab -e 'bash -c "sleep 5;rosservice call /uav1/mavros/set_message_interval 31 5000; exit; exec bash"' \
--tab -e 'bash -c "sleep 3;roslaunch fast_lio mapping_mid360_airsim.launch; exit;exec bash"' \
--tab -e 'bash -c "sleep 4;roslaunch prometheus_airsim mid360_to_octomap_airsim.launch; exit;exec bash"' \
--tab -e 'bash -c "sleep 4;roslaunch prometheus_airsim sitl_ego_octomap_mid360_airsim.launch; exit;exec bash"' \

