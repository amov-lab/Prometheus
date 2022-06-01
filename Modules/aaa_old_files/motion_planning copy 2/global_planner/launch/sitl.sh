# 启动Gazebo仿真环境
roslaunch prometheus_global_planner sitl_planning_p450_2dlidar.launch & sleep 5s;
# 启动
roslaunch prometheus_uav_control uav_control_main_outdoor.launch & sleep 1s;
# 启动
roslaunch prometheus_global_planner sitl_astar_2dlidar.launch

# 2dlidar需要联合ｏｃｔｏｍａｐ来做
# 如果使用ｍａｐｇｅｎｅｒａｔｏｒ则传感器数据由ｍａｐ产生，但是可以用ＰＸ４自身的动力学
