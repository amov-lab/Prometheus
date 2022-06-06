# 脚本名称: astar_with_global_pcl
# 脚本描述: 单个无人机的astar算法测试(局部点云)

# 启动Gazebo仿真环境 - Gazebo、rviz、map_generator_node
roslaunch prometheus_simulator_utils map_generator.launch & sleep 5s;
# 启动PX4 SITL 
roslaunch prometheus_gazebo sitl_px4_indoor.launch uav_init_x:=-10 & sleep 3s;
# 启动无人机控制 - uav_control_main_indoor
roslaunch prometheus_uav_control uav_control_main_indoor.launch & sleep 3s;
# 启动规划算法 
roslaunch prometheus_global_planner sitl_global_planner_with_local_point.launch



