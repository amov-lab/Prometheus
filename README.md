<img src="https://s1.ax1x.com/2020/04/12/GOesNq.jpg" alt="amov logo" align="right" height="60" />

# Prometheus - 自主无人机开源项目

**Prometheus**，在希腊神话中，是最具智慧的神明之一，希望本项目能为无人机研发工作带来无限的智慧与光明。

## 项目总览

Prometheus是一套开源的**自主无人机软件平台**，为无人机的智能与自主飞行提供**全套解决方案**。本项目基于**PX4**开源飞控固件，旨在为PX4开发者配套成熟可用的**机载电脑端**程序，提供更加简洁快速的开发体验。目前已集成**建图**、**定位**、**规划**、**控制**及**目标检测**等模块，并配套有Gazebo仿真测试代码。

 - **安装及使用：** [Prometheus开发者手册](https://github.com/amov-lab/Prometheus_User_Guide)

   - [Prometheus项目安装及编译](https://github.com/amov-lab/Prometheus_User_Guide/blob/master/%E5%BC%80%E5%8F%91%E8%80%85%E6%89%8B%E5%86%8C%20-%20%E5%AE%89%E8%A3%85%E5%8F%8A%E7%BC%96%E8%AF%91.md)

   - [Prometheus项目仿真模块环境搭建(Gazebo)](https://github.com/amov-lab/Prometheus_User_Guide/blob/master/%E5%BC%80%E5%8F%91%E8%80%85%E6%89%8B%E5%86%8C%20-%20Gazebo%E4%BB%BF%E7%9C%9F%E7%8E%AF%E5%A2%83%E5%AE%89%E8%A3%85%E5%8F%8A%E6%B5%8B%E8%AF%95.md)
   - 先看上面安装和环境搭建,再看具体的demo教程

请添加微信qyp0210进入内测交流群。

 - **配套硬件购买：**    即将推出，敬请期待！ 淘宝关注搜索 阿木实验室


**开源项目，维护不易，还烦请点一个star收藏，谢谢支持！**

## 功能展示
 - **RGBD相机建图**
 	
    <img width="400" src="https://s1.ax1x.com/2020/04/08/GWhyFO.gif"/>
 - **3D激光雷达建图(暂无演示图片)**

 - **局部规划(APF)**
	
    <img width="400" src="https://s1.ax1x.com/2020/04/12/GLvcpq.gif"/>
 - **全局规划(A star)**
 	
    <img width="400" src="https://s1.ax1x.com/2020/04/12/GOSrAe.gif"/>
 - **轨迹优化(Fast_Planner)**

	<img width="400" src="https://s1.ax1x.com/2020/04/12/GOCvHf.md.png"/>

- **外环控制器二次开发**
	
    <img width="400" src="https://s1.ax1x.com/2020/04/12/GOADfS.gif"/>
    
- **多机编队飞行**
  
    <img width="400" src="https://s1.ax1x.com/2020/04/12/GOAqmR.gif"/>
    
- **圆形穿越**

	<img width="400" src="https://s1.ax1x.com/2020/04/12/GOFrAf.gif"/>

- **颜色巡线**

	<img width="400" src="https://s1.ax1x.com/2020/04/12/GOkLi8.gif"/>
- **数字识别**

	<img width="400" src="https://s1.ax1x.com/2020/04/12/GOkVbt.gif"/>
- **KCF框选追踪**

	<img width="400" src="https://s1.ax1x.com/2020/04/12/GOkhxH.gif"/>
- **YOLO通用目标检测(暂缺演示图)**

- **移动平台自主降落**
	<img width="400" src="https://s1.ax1x.com/2020/08/11/aLUDAK.gif"/>

## 开发团队

 - 项目负责人
	 - [戚煜华](https://github.com/potato77) 
	 - [潇齐](https://github.com/orgs/amov-lab/people/amovlab-owl)
 - 建图模块
	 - [李春雨](https://github.com/Lee0326)
 - 路径规划及轨迹优化
	 - [江涛](https://github.com/orgs/amov-lab/people/panpanyunshi)
 - 目标识别及检测
	- [金忍](https://github.com/jario-jin)
 - 无人机控制及任务模块
	- [戚煜华](https://github.com/potato77) 
 - Gazebo仿真模块
	- [李春雨](https://github.com/Lee0326)
	- [戚煜华](https://github.com/potato77) 

## 版权申明

 - 本项目受 BSD 3-Clause 协议保护。点击LICENSE 了解更多
 - 本项目仅限个人使用，请勿用于商业用途。
 - 如利用本项目进行营利活动，阿木实验室将追究侵权行为。
