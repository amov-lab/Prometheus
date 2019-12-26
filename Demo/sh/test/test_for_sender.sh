##
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun prometheus_control fake_vicon; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_control px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_control px4_sender.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun prometheus_control move; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun prometheus_control set_mode; exec bash"' \
