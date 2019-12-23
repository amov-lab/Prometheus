##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command three_uav_mavros_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun px4_command formation_control_sitl; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun px4_command move; exec bash"' \

