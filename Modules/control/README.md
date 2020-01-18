# prometheus_control

## 简介

prometheus_control为Prometheus项目的控制模块。依赖Mavros功能包及prometheus_msgs功能包。

- 提供以下控制接口:
    - 怠速
    - 起飞
    - 悬停
    - 降落
    - 移动 (包括定点,定速及其复合模式,以及轨迹追踪)
 
- 可自定义位置环控制器

- 提供一系列位置控制测试代码


## 编译

若仅需要进行无人机控制相关的功能开发,可以仅编译本功能。

    `cd ~/Prometheus`
  
    `./compile_control.sh`


## 仿真环境运行

    `cd ~/Prometheus/Simulator/control_simulation`
  
    `./sitl_gazebo_px4_control.sh`

    
## 代码框架及接口说明

~/include 代码头文件
~/launch 启动脚本及参数配置文件
~/src    代码源文件

其中,**px4_pos_controller.cpp**为核心控制文件,应重点阅读。


 - 订阅命令话题 
 
   话题名称: "/prometheus/control_command"
   话题类型: prometheus_msgs::ControlCommand

**px4_pos_estimator.cpp**为位置估计器,应重点阅读。

发布无人机运动轨迹，话题为/prometheus/drone_trajectory，可通过参数pos_estimator/state_fromposehistory_window来设置轨迹的长短

## 关于作者

阿木实验室科研无人机技术负责人、Mavros培训课程主讲老师、北理工博士

对任何与四旋翼无人机有关的话题感兴趣，欢迎交流。

个人微信号：qyp0210
