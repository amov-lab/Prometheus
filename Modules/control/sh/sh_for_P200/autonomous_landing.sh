##circle tracking
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command tfmini.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch rplidar_ros rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch cartographer_ros rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch landpad_det landpad_det.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command autonomous_landing.launch; exec bash"' \

