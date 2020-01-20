# Planning

Planning模块从香港科技大学的fast-planner框架裁剪而来，可以实现四旋翼无人机快速自主飞行。
框架前端kinodynamic路径搜索，后端采用基于样条的轨迹生成，同时还包含了时间调节系统。
Fast-planner可以在及其短的时间内（几毫秒）生成高质量轨迹。

>参考文献  
>[__Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight__](https://ieeexplore.ieee.org/document/8758904), Boyu Zhou, Fei Gao, Luqi Wang, Chuhao Liu and Shaojie Shen, IEEE Robotics and Automation Letters (RA-L), 2019.


## 1. 安装

- 软件开发环境为 Ubuntu 16.04, [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).
>编译  
* cd Prometheus/  
* ./compile_control_planning.sh

## 3. Run the Simulation


## 4. 致谢
  * 使用 **nlopt**作为非线性优化工具 （位于/ThirdParty）
  * 感谢港科的老师、同学！！  

## 5. 说明
* 与控制接口  plan_manage/src/traj_server.cpp
> msgs/msg/PlanningPositionCommand.msg


