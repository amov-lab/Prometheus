# local planner  
## 1. 原理
* apf方法  
* 在 planning_simulator.launch 调节对应的算法参数

## 2. 运行  
* 如果使用planner自带仿真，运行
> roslaunch prometheus_local_planning planning_simulator.launch  
> roslaunch prometheus_local_planning rviz.launch

* 如果使用px4仿真运行
> roslaunch prometheus_local_planning planning_px4.launch  
> roslaunch prometheus_local_planning rviz.launch  

### ctrl+shift+v 预览 