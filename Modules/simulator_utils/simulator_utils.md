## simulator_utils

本模块为仿真组件模块，可用于辅助路径规划仿真，功能包括
 - 点云地图生成(map_generator)
 - 虚拟无人机(fake_odom)
 - rviz辅助显示(rviz_visualization)

#### map_generator
map_generator节点可用于模拟真实环境中的感知信息，生成用于路径规划的全局点云和局部点云地图。

map_generator.cpp和map_genrator_node.cpp：
 - 环境基本形状包括：矩形块、圆柱体、墙体、线条等（可通过参数配置基本形状的参数）
 - 内置planning_test，planning_test2，random，test四种地图（可通过参数配置选择地图种类或自行构建地图，PS：如果需要gazebo显示，自行构建的地图需要自行构建gazebo环境），map_name和gazebo world对应关系如下
  - planning_test 对应 planning_test.world
  - planning_test2 对应 planning_test2.world
 - 定时发布全局点云
  - 话题名称：/map_generator/global_cloud
  - 话题类型：sensor_msgs::PointCloud2
  - 发布频率可通过参数配置，默认为1Hz
 - 定时发布局部点云
  - 支持同时发布多架无人机局部点云信息，数量在通过参数配置
  - 发布局部点云需提供无人机的odom信息，订阅话题为：/uavid/prometheus/odom
  - 话题名称：/uavid/map_generator/local_cloud
  - 话题类型：sensor_msgs::PointCloud2
  - 发布频率可通过参数配置，默认为10Hz
 - 目前laser_sim_node.cpp仍在测试阶段，功能为模拟激光雷达数据
 
测试：
roslaunch prometheus_simulator_utils map_generator.launch

 - 启动后可以在rivz中看到绿色的全局点云
 - map_name为地图名称，注意选择与地图对应的gazebo world
 - 如果不需要启动gazebo显示，可以将gazebo_enable设置为false
 - swarm_num为无人机数量，默认为1
 - 与地图相关的参数可以在map_generator.cpp文件中修改，也可以引出到launch文件中修改，具体说明见代码注释
 - 此时由于没有发布无人机odom，因此在rviz中只能订阅到全局点云信息，而没有局部点云信息

#### fake_odom
fake_odom节点可用于模拟真实环境中的无人机运动学模型（在仿真中起到替换PX4 SITL的目的，即不需要启动PX4仿真），订阅来自控制模块的底层控制指令，并模拟生成无人机odom信息。（强调：仅是运动学模拟，即假设无人机为质点）

fake_uav.cpp和fake_odom_node.cpp：
 - 参数说明
  - fake_odom/swarm_num_uav ： 无人机数量
  - todo...
 - 订阅话题
  - 话题名称：xx
  - 话题类型：xx
 - 发布话题
  - 话题名称：xx
  - 话题类型：xx

测试（两个终端依次启动）：
roslaunch prometheus_simulator_utils map_generator.launch
roslaunch prometheus_simulator_utils fake_odom.launch
 - 启动后第一个终端后可以在rivz中看到绿色的全局点云
 - 启动后第二个终端后可以在rivz中看到红色的局部点云

#### quadrotor_dynamics
 无人机动力学模型模拟，开发ing

#### rviz_visualization
 todo

