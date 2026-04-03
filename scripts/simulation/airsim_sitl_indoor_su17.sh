# 脚本名称: px4_sitl_outdoor
# 脚本描述: PX4 SITL Gazebo仿真环境测试

gnome-terminal --window -e 'bash -c "roscore; exit; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_airsim sitl_px4_indoor_airsim.launch;exit; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_indoor_mid360.launch;exit; exec bash"' \
--tab -e 'bash -c "sleep 6; rosservice call /uav1/mavros/set_message_interval 31 5000;exit; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$PX4_SIM_HOST_ADDR; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch fast_lio mapping_mid360_airsim.launch; exec bash"' \
