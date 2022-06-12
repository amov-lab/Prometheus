# 脚本名称: ego_planner_4uav
# 脚本描述: ego_planner算法测试
# 本脚本启动后,需要启动 roslaunch ego_planner sitl_ego_planner_pub_goal.launch 发布目标点

# 启动Gazebo仿真环境 - Gazebo、rviz、map_generator_node
roslaunch prometheus_simulator_utils map_generator.launch swarm_num:=4 & sleep 5s;
# 启动PX4 SITL 
roslaunch prometheus_gazebo sitl_indoor_4uav.launch gazebo_enable:=false uav1_init_x:=-10 uav1_init_y:=-10 uav2_init_x:=-10 uav2_init_y:=-5 uav3_init_x:=-10 uav3_init_y:=5 uav4_init_x:=-10 uav4_init_y:=10 & sleep 5s;
# 启动无人机控制 - uav_control_main_indoor
roslaunch prometheus_uav_control uav_control_main_indoor_4uav.launch & sleep 4s;
# 启动无人机规划 - ego_planner、traj_server
roslaunch ego_planner sitl_ego_planner_4uav.launch


