# 脚本名称: ego_planner_1uav
# 脚本描述: 单个无人机的ego_planner算法测试 - scan

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_simulator_utils map_generator.launch map_generator_enable:=false; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_gazebo sitl_indoor_1uav.launch gazebo_enable:=false uav1_init_x:=-10; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_uav_control uav_control_main_indoor.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch ego_planner sitl_ego_planner_basic.launch; exec bash"' \