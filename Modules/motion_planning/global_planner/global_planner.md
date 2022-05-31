## global_planner

  - 使用gazebo环境获取感知信息还是通过map_generator?
  - 使用PX4自带的动力学or fake_odom模块?
  - 

#### 情况讨论

 - 全局点云情况
   - 真实场景对应:已知全局地图
   - map_generator生成全局点云
   - 无人机不需要搭载传感器
   - PX4动力学 & fake_odom模块
 
roslaunch prometheus_simulator_utils map_generator_with_fake_odom.launch
roslaunch prometheus_global_planner sitl_global_planner_with_global_point.launch


 - 局部点云情况
   - 对应真实情况:D435i等RGBD相机,三维激光雷达
   - map_generator生成全局点云,模拟得到局部点云信息
   - 无人机不需要搭载传感器
   - PX4动力学 & fake_odom模块

roslaunch prometheus_simulator_utils map_generator_with_fake_odom.launch
roslaunch prometheus_global_planner sitl_global_planner_with_local_point.launch

 - 2dlidar情况
   - 对应真实情况:二维激光雷达
   - projector_.projectLaser(*local_point, input_laser_scan)将scan转换为点云,即对应回到局部点云情况
   - map_generator生成全局点云,模拟得到局部点云信息
   - 无人机不需要搭载传感器
   - PX4动力学 & fake_odom模块

roslaunch prometheus_simulator_utils map_generator_with_fake_odom.launch
roslaunch prometheus_global_planner sitl_global_planner_with_2dlidar.launch

 - 多机情况
   - 建议使用 全局点云情况 + 多个无人机
   - fake_odom模块

## 运行

