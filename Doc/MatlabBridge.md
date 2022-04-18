#### matlab_bridge

- 启动仿真 - 单机 - 无需遥控器

终端１：
roslaunch prometheus_gazebo sitl_outdoor.launch
终端２：
roslaunch prometheus_uav_control matlab_bridge_with_uav_control_no_rc.launch 

- 启动仿真 - 多机