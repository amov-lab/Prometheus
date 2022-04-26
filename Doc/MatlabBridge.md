#### matlab_bridge


- IMU话题
 类型:sensor_msgs/Imu
 话题名:/uav1/mavros/imu/data
- 位置话题
 类型:nav_msgs/Odometry
 话题名:/uav1/prometheus/ground_truth
  

- 启动仿真 - 单机 - 无需遥控器

终端１：
roslaunch prometheus_gazebo sitl_outdoor.launch
终端２：
roslaunch prometheus_matlab matlab_bridge_with_uav_control_no_rc.launch 

- 启动仿真 - 多机