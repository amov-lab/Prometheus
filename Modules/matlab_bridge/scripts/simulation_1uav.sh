
# 启动Gazebo仿真环境, px4 sitl, mavros
roslaunch prometheus_gazebo sitl_indoor.launch & sleep 5s;
# 启动无人机控制
roslaunch prometheus_uav_control uav_control_main_indoor.launch flag_printf:=false & sleep 1s;
# 启动matlab_bridge
roslaunch prometheus_matlab matlab_bridge.launch 
