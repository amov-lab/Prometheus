#### matlab_bridge


- IMU话题
 类型:sensor_msgs/Imu
 话题名:/uav1/mavros/imu/data
- 位置话题
 类型:nav_msgs/Odometry
 话题名:/uav1/prometheus/ground_truth
  

- 启动仿真 - 单机 - 需遥控器

cd Prometheus/Modules/matlab_bridge/scripts/
./simulation_1uav.sh 

- 启动仿真 - 多机 - 需遥控器

cd Prometheus/Modules/matlab_bridge/scripts/
./simulation_4uav.sh 