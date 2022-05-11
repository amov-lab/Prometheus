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


// 1、PX4连接状态
// 2、是否解锁
// 3、是否处于OFFBOARD模式
// 4、odom是否有效
// 5、uav_control是否处于COMMAND_CONTROL模式 （HOVER）

REJECT = 1,
SUCCESS = 2,
HEARTBEAT = 3,
MANUAL = 4

1-5都OK的话，你check一次，我才会返回success
1-5都OK的话，其余时间发HEARTBEAT
1-4都OK的话，其余时间发MANUAL