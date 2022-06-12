# 脚本名称: px4_sitl_indoor
# 脚本描述: PX4 SITL Gazebo仿真环境测试

# 启动PX4 SITL及Gazebo仿真环境
roslaunch prometheus_gazebo sitl_indoor_1uav.launch & sleep 5s;
# 启动无人机控制 - uav_control_main
roslaunch prometheus_uav_control uav_control_main_indoor.launch



