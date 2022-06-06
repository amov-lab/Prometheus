# 脚本名称: uav_controller_ne
# 脚本描述: 无人机位置环控制器测试

# 启动PX4 SITL及Gazebo仿真环境
roslaunch prometheus_gazebo sitl_indoor_1uav.launch & sleep 5s;
# 启动无人机控制 - uav_control_main
roslaunch prometheus_uav_control uav_control_main_controller_test.launch controller_flag:=3



