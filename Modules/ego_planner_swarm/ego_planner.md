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

cd Prometheus/Scripts/simulation/ego_planner/
单机版本
./ego_planner_1uav.sh
四机版本
./ego_planner_4uav.sh

飞机加载完毕后，检查报错，然后解锁-切换至COMMAND_CONTROL模式，无人机自动起飞

然后，给定目标点即可

## 如何发布目标点

 - 使用RVIZ的插件
 - 使用rosrun ego_planner pub_goal 发布任意目标点
    - x和y都等于99.99时为特殊指令，无人机原地悬停，等待下一个目标点
 - 使用roslaunch ego_planner pub_preset_goal.launch 发布预设的目标点（默认的目标点可以在launch文件中修改，目前默认是四架飞机）


## 注意事项

 - 在规划过程中（没有抵达目标点的时候），可以切换至RC_POS_CONTROL，但如果切换回COMMAND_CONTROL模式，可能会发生碰撞。正确的做法是发布一个悬停目标点（99.99）。