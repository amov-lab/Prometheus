##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_control SITL.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_control px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_control px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun prometheus_control move; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun prometheus_control set_mode; exec bash"' \

