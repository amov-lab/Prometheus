# 脚本名称: px4_sitl_outdoor
# 脚本描述: PX4 SITL Gazebo仿真环境测试

gnome-terminal --window -e 'bash -c "roscore; exit; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_airsim sitl_px4_outdoor_airsim.launch;exit; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_outdoor.launch;exit; exec bash"' \
