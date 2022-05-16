#### 无人机demo仿真手册

## 运行无人机demo仿真前需确保无人机仿真能正常运行,详情请查看Prometheus/Doc/Gazebo仿真手册.md相关内容

## 运行无人机demo仿真需配备有遥控器,详情请查看Prometheus/Doc/虚拟摇杆使用.md以及遥控器说明.md

## 运行无人机demo仿真需要将遥控器通过USB接口连接到仿真电脑


*** Basic ***

- 起飞降落

# launch文件
roslaunch prometheus_gazebo sitl_outdoor_1uav.launch
roslaunch prometheus_uav_control uav_control_main_outdoor.launch
roslaunch prometheus_demo takeoff_land.launch

# 遥控器操作
1. 将SWA档杆拨到最下端位置,无人机将解锁
2. 将SWB档杆拨到中间位置短暂停留后拨到最下端位置,无人机将起飞,完成任务后自动降落


- 本地坐标系(ENU)下的位置控制

# launch文件
roslaunch prometheus_gazebo sitl_outdoor_1uav.launch
roslaunch prometheus_uav_control uav_control_main_outdoor.launch
roslaunch prometheus_demo enu_xyz_pos_control.launch

# 遥控器操作
1. 将SWA档杆拨到最下端位置,无人机将解锁
2. 将SWB档杆拨到中间位置短暂停留后拨到最下端位置,无人机将起飞,完成任务后自动降落

- 机体坐标系(BODY)下的位置控制

# launch文件
roslaunch prometheus_gazebo sitl_outdoor_1uav.launch
roslaunch prometheus_uav_control uav_control_main_outdoor.launch
roslaunch prometheus_demo body_xyz_pos_control.launch

# 遥控器操作
1. 将SWA档杆拨到最下端位置,无人机将解锁
2. 将SWB档杆拨到中间位置短暂停留后拨到最下端位置,无人机将起飞,完成任务后自动降落

- 全局坐标系(WGS84)下的经纬度控制

# launch文件
roslaunch prometheus_gazebo sitl_outdoor_1uav.launch
roslaunch prometheus_uav_control uav_control_main_outdoor.launch
roslaunch prometheus_demo global_pos_control.launch

# 遥控器操作
1. 将SWA档杆拨到最下端位置,无人机将解锁
2. 将SWB档杆拨到中间位置短暂停留后拨到最下端位置,无人机将起飞,完成任务后自动降落

- 圆形跟踪(本地坐标系下的速度控制)

# launch文件
roslaunch prometheus_gazebo sitl_outdoor_1uav.launch
roslaunch prometheus_uav_control uav_control_main_outdoor.launch
roslaunch prometheus_demo circular_trajectory_control.launch

# 遥控器操作
1. 将SWA档杆拨到最下端位置,无人机将解锁
2. 将SWB档杆拨到中间位置短暂停留后拨到最下端位置,无人机将起飞,完成任务后自动降落

*** Advanced ***

- 集群控制

# launch文件
roslaunch prometheus_gazebo sitl_outdoor_4uav.launch
roslaunch prometheus_uav_control uav_control_main_outdoor_4uav.launch
roslaunch prometheus_demo formation_control.launch

# 遥控器操作

1. 待无人机全部启动正常后,在无人机集群控制终端窗口输入1启动无人机集群控制
2. 将SWA档杆拨到最下端位置,无人机集群将解锁
3. 将SWB档杆拨到中间位置短暂停留后拨到最下端位置,无人机将起飞,左侧摇杆向上推,将无人机集群升起一定高度后,将SWB拨杆拨到最下端位置
4. 在终端窗口输入选项以及对应控制
    0 : 位置控制,输入0后按下回车,依次输入XYZ的值并按下回车(未设置队形的情况下默认将会以一字队形飞行)
    1 : 切换为一字队形
    2 : 切换为三角队形

- 自定义控制器测试

- 多二维码识别

- 二维码引导降落

- 目标锁定追踪