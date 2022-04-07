gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_formation_5uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; rosrun prometheus_mission formation_state; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo prometheus_formation_control.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_gazebo prometheus_formation_setmode.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch prometheus_gazebo prometheus_formation_change.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun prometheus_mission formation_move; exec bash"' \
