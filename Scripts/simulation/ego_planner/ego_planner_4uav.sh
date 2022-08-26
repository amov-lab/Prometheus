# 脚本名称: ego_planner_4uav
# 脚本描述: ego_planner算法测试
# 本脚本启动后,需要启动 roslaunch ego_planner sitl_ego_planner_pub_goal.launch 发布目标点

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_simulator_utils map_generator.launch swarm_num:=4; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_gazebo sitl_indoor_4uav.launch gazebo_enable:=false uav1_init_x:=-10 uav1_init_y:=-3 uav2_init_x:=-10 uav2_init_y:=-1 uav3_init_x:=-10 uav3_init_y:=1 uav4_init_x:=-10 uav4_init_y:=3; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_uav_control uav_control_main_indoor_4uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch ego_planner sitl_ego_planner_4uav.launch; exec bash"' \