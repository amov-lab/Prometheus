## global_planner

  - 使用gazebo环境获取感知信息还是通过map_generator?
  - 使用PX4自带的动力学or fake_odom模块?
  - 

#### 情况讨论

 - 全局点云情况(测试)
   - 真实场景对应:已知全局地图
   - map_generator生成全局点云
   - 无人机不需要搭载传感器
   - PX4动力学 & fake_odom模块
 

启动仿真环境
roslaunch prometheus_simulator_utils map_generator_with_fake_odom.launch 
 
 - 依次拨动swb，切换值COMMAND_CONTROL模式，无人机会自动起飞至指定高度

启动规划算法
roslaunch prometheus_global_planner sitl_global_planner_with_global_point.launch
 
 - 在RVIZ中选定目标点，或者使用终端发布rostopic pub /uav1/prometheus/motion_planning/goal xxx



 - 局部点云情况（todo）
   - 对应真实情况:D435i等RGBD相机,三维激光雷达
   - map_generator生成全局点云,模拟得到局部点云信息
   - 无人机不需要搭载传感器
   - PX4动力学 & fake_odom模块


启动仿真环境
roslaunch prometheus_simulator_utils map_generator_with_fake_odom.launch 
 
 - 依次拨动swb，切换值COMMAND_CONTROL模式，无人机会自动起飞至指定高度

启动规划算法
roslaunch prometheus_global_planner sitl_global_planner_with_local_point.launch
 
 - 在RVIZ中选定目标点，或者使用终端发布rostopic pub /uav1/prometheus/motion_planning/goal xxx


 - 2dlidar情况（todo）
   - 对应真实情况:二维激光雷达
   - projector_.projectLaser(*local_point, input_laser_scan)将scan转换为点云,即对应回到局部点云情况
   - map_generator生成全局点云,模拟得到局部点云信息
   - 无人机不需要搭载传感器
   - PX4动力学 & fake_odom模块

启动仿真环境
roslaunch prometheus_simulator_utils map_generator_with_fake_odom.launch 
 
 - 依次拨动swb，切换值COMMAND_CONTROL模式，无人机会自动起飞至指定高度

启动规划算法
roslaunch prometheus_global_planner sitl_global_planner_with_2dlidar.launch
 
 - 在RVIZ中选定目标点，或者使用终端发布rostopic pub /uav1/prometheus/motion_planning/goal xxx


 - 多机情况（todo）
   - 建议使用 全局点云情况 + 多个无人机
   - fake_odom模块

## wiki安排

 6.1 simulator_utils
  6.1.1 map_generator
  6.1.2 fake_odom
 6.2 motion_planning
  6.2.1 global_planner
  6.2.2 local_planner
 6.3 ego_planner


