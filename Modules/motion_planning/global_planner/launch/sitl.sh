# 启动Gazebo仿真环境
roslaunch prometheus_simulator_utils map_generator_with_fake_odom.launch & sleep 5s;
# 启动
roslaunch prometheus_global_planner sitl_global_planner_with_global_point.launch

# roslaunch prometheus_global_planner sitl_global_planner_with_2dlidar.launch

# 2dlidar需要联合ｏｃｔｏｍａｐ来做
# 如果使用ｍａｐｇｅｎｅｒａｔｏｒ则传感器数据由ｍａｐ产生，但是可以用ＰＸ４自身的动力学
