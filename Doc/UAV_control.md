#### uav_control

- 原始指令读取
  - ubuntu系统中使用jstest-gtk命令
  - win系统中使用遥控器驱动软件

- 遥控器指令测试
roslaunch prometheus_gazebo sitl_outdoor.launch
roslaunch prometheus_uav_control rc_test.launch 
