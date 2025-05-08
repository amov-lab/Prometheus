# 脚本名称: px4_sitl_outdoor
# 脚本描述: PX4 SITL Gazebo仿真环境测试

gnome-terminal --window -e 'bash -c "roscore; exit; exec bash"' \
--tab -e 'bash -c "source ~/prometheus_px4/Tools/setup_gazebo.bash ~/prometheus_px4 ~/prometheus_px4/build/amovlab_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4/Tools/sitl_gazebo;sleep 3; roslaunch prometheus_airsim sitl_px4_outdoor_airsim_3uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_outdoor_3uav.launch; exec bash"' \

