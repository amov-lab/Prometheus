# 脚本名称: ego_planner_1uav_gfkd
# 脚本描述: 国防科大单机ego_planner算法测试 - scan

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch prometheus_gfkd gazebo_world.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_gfkd main.launch; exec bash"' \
