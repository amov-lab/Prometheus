# 脚本名称: body_xyz_pos_control
# 脚本描述: 该脚本为机体坐标系下控制demo启动脚本,包含PX4 SITL,Gazebo仿真环境,无人机控制节点以及机体坐标系位置控制节点

# 启动PX4 SITL及Gazebo仿真环境
roslaunch prometheus_gazebo sitl_outdoor_1uav.launch & sleep 5s;
# 启动无人机控制 - uav_control_main
roslaunch prometheus_uav_control uav_control_main_outdoor.launch & sleep 5s;
# 启动demo (C语言版本)
roslaunch prometheus_demo body_xyz_pos_control.launch
# 启动demo (python版本)
# rosrun prometheus_demo body_xyz_pos_control.py