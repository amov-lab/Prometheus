##动捕集群启动脚本
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo sitl_formation_4uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun prometheus_mission formation_state; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_gazebo prometheus_formation_square.launch; exec bash"' \



