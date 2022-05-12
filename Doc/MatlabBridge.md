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

 matlab_setting_result.x
// 0b 000000
// bit1: connected or not
// bit2: odom_valid or not
// bit3: armed or not
// bit4: OFFBOARD or not
// bit5 & 6: OTHER(00) MANUAL(01) or COMMAND_MODE(10)


- 遥控器控制无人机切换到command模式，无人机起飞后再打开matlab程序
- 如果实验中途想修改matlab代码
    - 无人机降落按钮，降落后，重新执行起飞操作，机载程序无需关闭
    - 无人机拨到HOVER_CONTROL模式，无人机会原地悬停，且遥控器可控，再次拨回COMMAND_CONTROL时，无人机会回到起始点重新开始
- 如果cmd timeout 无人机依次判断执行原地悬停等待（MATLAB_CMD_TIMEOUT）-> 返回起始点（RETURN_INIT_POS_TIMEOUT）-> 降落（LAND_TIMEOUT）
- 终止实验，使用遥控器的land按钮