# 脚本名称: px4_sitl_indoor
# 脚本描述: PX4 SITL Gazebo仿真环境测试

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo sitl_indoor_1uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_indoor.launch; exec bash"' \