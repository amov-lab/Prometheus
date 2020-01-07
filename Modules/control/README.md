# prometheus_control

## 简介

prometheus_control功能包是Prometheus项目中负责底层控制的功能包。


## 编译

若仅需要进行无人机控制相关的功能开发,可以仅编译本功能。(不会编译目标检测,SLAM,路径规划等其他功能包)

    `cd ~/Prometheus`
  
    `./compile_control.sh`


## 仿真环境运行

    `cd ~/Prometheus/Simulator/control_simulation`
  
    `./sitl_gazebo_px4_control.sh`

## 真实飞行


    
## 代码框架及接口说明


 - 订阅命令话题 
 
   话题名称: "/prometheus/control_command"
   话题类型: prometheus_msgs::ControlCommand


## 关于作者

阿木实验室科研无人机技术负责人、Mavros培训课程主讲老师、北理工博士

对任何与四旋翼无人机有关的话题感兴趣，欢迎交流。

个人微信号：qyp0210
