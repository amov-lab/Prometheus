# 脚本名称: future_aircraft_demo
# 脚本描述: future_aircraft_demo未来飞行器仿真测试

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_future_aircraft future_aircraft.launch; exec bash"' \
gnome-terminal --window -e 'bash -c "sleep 6; rosrun prometheus_future_aircraft future_aircraft; exec bash"' \
