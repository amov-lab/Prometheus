
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch digitnum_det_ros num_det.launch; exec bash"' \
