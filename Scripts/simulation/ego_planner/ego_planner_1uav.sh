# 脚本名称: ego_planner_1uav
# 脚本描述: 单个无人机的ego_planner算法测试

# 启动Gazebo仿真环境 - Gazebo、rviz、map_generator_node
roslaunch prometheus_simulator_utils map_generator.launch swarm_num:=1 & sleep 5s;
# 启动PX4 SITL 
roslaunch prometheus_gazebo sitl_px4_indoor.launch uav_init_x:=-10 & sleep 5s;
# 启动无人机控制 - uav_control_main_indoor
roslaunch prometheus_uav_control uav_control_main_indoor.launch & sleep 4s;
# 启动无人机规划 - ego_planner、traj_server
roslaunch ego_planner sitl_ego_planner_basic.launch 


