##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch px4 posix_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch prometheus_control px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch prometheus_control px4_pos_controller.launch; exec bash"' \
--window -e 'bash -c "sleep 1; roslaunch prometheus_control terminal_control.launch; exec bash"' \
--window -e 'bash -c "sleep 1; rosrun prometheus_control ground_station; exec bash"' \
