# 启动Gazebo仿真环境
roslaunch prometheus_simulator_utils map_generator_with_fake_odom.launch & sleep 5s;
# 启动
roslaunch prometheus_local_planner sitl_apf_with_local_point.launch
