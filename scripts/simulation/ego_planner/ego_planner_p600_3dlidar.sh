# 脚本描述: 

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_outdoor_1uav_P600_mid360.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch prometheus_uav_control uav_control_main_outdoor.launch; exec bash"' \


