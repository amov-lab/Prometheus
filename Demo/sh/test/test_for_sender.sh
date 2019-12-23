##
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_command fake_vicon; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command px4_sender.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_command move; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_command set_mode; exec bash"' \
