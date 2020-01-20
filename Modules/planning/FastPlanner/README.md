# Planning

Planning模块从香港科技大学的fast-planner框架裁剪而来，可以实现四旋翼无人机快速自主飞行。
框架前端kinodynamic路径搜索，后端采用基于样条的轨迹生成，同时还包含了时间调节系统。
Fast-planner可以在及其短的时间内（几毫秒）生成高质量轨迹(依赖增量式sdf地图构建)。

>参考文献  
>[__Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight__](https://ieeexplore.ieee.org/document/8758904), Boyu Zhou, Fei Gao, Luqi Wang, Chuhao Liu and Shaojie Shen, IEEE Robotics and Automation Letters (RA-L), 2019.


## 1. 安装

- 软件开发环境为 Ubuntu 16.04, [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).
- 非线性优化工具箱 [**NLopt**](https://nlopt.readthedocs.io/en/latest/NLopt_Installation)
>编译  
* cd Prometheus/  
* ./compile_control_planning.sh

## 3. Run the Simulation
$ roslaunch prometheus_plan_manage prometheus_planning_test_static.launch  
 > 修改 pcd_file 为自己的配置参数  

$ roslaunch prometheus_plan_manage rviz_static.launch  



## 4. 致谢
  * 使用 **nlopt**作为非线性优化工具 （位于/ThirdParty）(https://nlopt.readthedocs.io/en/latest/NLopt_Installation)  
  * 使用 **sdf_tool**为地图转化工具

## 5. 说明
* 与控制接口  plan_manage/src/traj_server.cpp  （未完，待补充）  
> msgs/msg/PositionReference.msg  

* 输入odom信息（topic: "/planning/odom_world"）  
* 输入pcd地图信息（目前地图只支持有限空间地图，地图大小、分辨率在launch文件设置，topic： "/planning/global_point_cloud"）  
* 从rviz输入需要的期望goal, 选择3d navigation, 同时按下鼠标左右键，然后上下移动标记z大小。  



