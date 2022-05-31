# 启动Gazebo仿真环境
roslaunch prometheus_global_planner sitl_indoor_1uav.launch & sleep 5s;
# 启动
roslaunch prometheus_global_planner sitl_global_planner_with_2dlidar.launch