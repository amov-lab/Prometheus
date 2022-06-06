## EGO_planner_swarm

map_generator
- 局部点云
- 全局点云

uav_control
- fake_odom
- PX4

#### 情况讨论

 - 局部点云情况
   - 对应真实情况:D435i等RGBD相机,三维激光雷达
   - map_generator生成全局点云,模拟得到局部点云信息
   - 无人机不需要搭载传感器
   - PX4动力学 & fake_odom模块

roslaunch prometheus_simulator_utils map_generator_with_fake_odom.launch
roslaunch ego_planner sitl_ego_planner_basic.launch


## 运行

