##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_control three_uav_mavros_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun prometheus_control formation_control_sitl; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun prometheus_control move; exec bash"' \

