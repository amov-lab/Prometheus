# px4_command

# 简介

px4_command功能包是一个基于PX4开源固件及Mavros功能包的开源项目，旨在为PX4开发者提供更加简洁快速的开发体验。目前已集成无人机外环控制器修改、目标追踪及避障等上层开发代码，后续将陆续推出涵盖任务决策、路径规划、滤波导航、单/多机控制等无人机科研及开发领域的功能。

# 安装
1. 通过二进制的方法安装Mavros功能包
   
    请参考: https://github.com/mavlink/mavros
    
   如果你已经使用源码的方式安装过Mavros功能包，请先将其删除。

2. 在home目录下创建一个名为 "px4_ws" 的工作空间

    `mkdir -p ~/px4_ws/src`
  
    `cd ~/px4_ws/src`
  
    `catkin_init_workspace`
    
      大部分时候，需要手动进行source，打开一个新终端 
    
    `gedit .bashrc`  
    
    在打开的`bashrc.txt`文件中添加 `source /home/$(your computer name)/px4_ws/devel/setup.bash`
 
 3. 下载并编译 `px4_command` 功能包
    
    `cd ~/px4_ws/src`

    `git clone https://github.com/potato77/px4_command`
    
    `cd ..`
    
    `catkin_make`

# 项目总览

# ![未命名表单](https://i.imgur.com/Au69Gps.png)

 - 读取飞控状态 [state_from_mavros.h](https://github.com/potato77/px4_command/blob/master/include/state_from_mavros.h "state_from_mavros.h")

 - 发送控制指令至飞控 [command_to_mavros.h](https://github.com/potato77/px4_command/blob/master/include/command_to_mavros.h "command_to_mavros.h")

- 位置环控制器实现（目前提供五种外环控制器，分别为串级PID、PID、UDE、Passivity+UDE、NE+UDE）
  [pos_controller_cascade_PID.h](https://github.com/potato77/px4_command/blob/master/include/pos_controller_cascade_PID.h "pos_controller_cascade_PID.h")
  [pos_controller_PID.h](https://github.com/potato77/px4_command/blob/master/include/pos_controller_PID.h "pos_controller_PID.h")
  [pos_controller_UDE.h](https://github.com/potato77/px4_command/blob/master/include/pos_controller_UDE.h "pos_controller_UDE.h")
  [pos_controller_Passivity.h](https://github.com/potato77/px4_command/blob/master/include/pos_controller_Passivity.h "pos_controller_Passivity.h")
  [pos_controller_NE.h](https://github.com/potato77/px4_command/blob/master/include/pos_controller_NE.h "pos_controller_NE.h")
  其中，串级PID为仿写PX4中位置控制器、Passivity+UDE为无需速度反馈的位置控制器、NE+UDE在速度测量有噪声时由于其他控制器。

- 外部定位实现 [px4_pos_estimator.cpp](https://github.com/potato77/px4_command/blob/master/src/px4_pos_estimator.cpp "px4_pos_estimator.cpp")
- 控制逻辑主程序 [px4_pos_controller.cpp](https://github.com/potato77/px4_command/blob/master/src/px4_pos_controller.cpp "px4_pos_controller.cpp")
- 地面站（需配合ROS多机使用） [ground_station.cpp](https://github.com/potato77/px4_command/blob/master/src/ground_station.cpp "ground_station.cpp") 
- 自主降落 [autonomous_landing.cpp](https://github.com/potato77/px4_command/blob/master/src/Application/autonomous_landing.cpp "autonomous_landing.cpp")
- 简易避障 [collision_avoidance.cpp](https://github.com/potato77/px4_command/blob/master/src/Application/collision_avoidance.cpp "collision_avoidance.cpp")
- 双目简易避障 [collision_avoidance_streo.cpp](https://github.com/potato77/px4_command/blob/master/src/Application/collision_avoidance_streo.cpp "collision_avoidance_streo.cpp")
- 编队飞行（目前仅支持gazebo仿真）[formation_control_sitl.cpp](https://github.com/potato77/px4_command/blob/master/src/Application/formation_control_sitl.cpp "formation_control_sitl.cpp")
- 负载投掷 [payload_drop.cpp](https://github.com/potato77/px4_command/blob/master/src/Application/payload_drop.cpp "payload_drop.cpp")
- 航点追踪 [square.cpp](https://github.com/potato77/px4_command/blob/master/src/Application/square.cpp "square.cpp")
- 目标追踪 [target_tracking.cpp](https://github.com/potato77/px4_command/blob/master/src/Application/target_tracking.cpp "target_tracking.cpp")

> 说明：
> 1、其中自主降落、目标追踪、双目简易避障需配合vision部分代码使用。
> 2、功能包中还包含一些简易滤波及测试小代码，此处不作说明，可自行查阅。

## 视频演示

[自主降落](https://www.bilibili.com/video/av60648116/)

[负载投掷](https://www.bilibili.com/video/av55037908/)

[简易避障、目标追踪](https://www.bilibili.com/video/av60648886/)

[外环控制器修改  NE+UDE](https://www.bilibili.com/video/av60963113/) 及 [外环控制器修改 Passivity-based UDE](https://www.bilibili.com/video/av60979252/)

[内环控制器修改（PX4固件）](https://www.bilibili.com/video/av60962814/)

## 坐标系说明
   
   本功能包中所有变量均为 **ENU** 坐标系（同Mavros，异于PX4固件）

  >  MAVROS does translate Aerospace NED frames, used in FCUs to ROS ENU frames and vice-versa. For translate airframe related data we simply apply rotation 180° about ROLL (X) axis. For local we apply 180° about ROLL (X) and 90° about YAW (Z) axes
  
## PX4固件及参数

请使用提供代码中提供的PX4固件 - [firmware](https://github.com/potato77/px4_command/tree/master/firmware "firmware")

SYS_COMPANION参数设置为Companion（921600）。

若需要使用外部定位，则参数EKF2_AID_MASK设为24（默认为1），EKF2_HGT_MODE设置为VISION（默认为气压计）。

## 其他教程

[随意写写](https://github.com/potato77/Tech_Blog)

## 多机Gazebo仿真

多机仿真前，请确保PX4固件能够顺利运行单机及双机仿真例程。

请先将该文件[iris_3](https://github.com/potato77/px4_command/blob/master/src/Application/iris_3)放置于PX4固件Firmware/posix-configs/SITL/init/ekf2/目录下（固件版本 v1.8.2），然后运行脚本[sitl_gazebo_formation](https://github.com/potato77/px4_command/blob/master/sh/sh_for_simulation/sitl_gazebo_formation.sh)即可。


# 远程桌面

推荐使用[nomachine](https://www.nomachine.com)作为远程桌面。

# 关于作者

阿木实验室科研无人机技术负责人、Mavros培训课程主讲老师、北理工博士

对任何与四旋翼无人机有关的话题感兴趣，欢迎交流。

个人微信号：qyp0210
