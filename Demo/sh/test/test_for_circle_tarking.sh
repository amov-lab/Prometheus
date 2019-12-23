##
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun prometheus_control fake_vicon; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_control px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_control px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun web_cam web_cam; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch ellipse_det ellipse_det.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch prometheus_control target_tracking.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun prometheus_control set_mode; exec bash"' \
