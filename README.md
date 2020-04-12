<img src="https://s1.ax1x.com/2020/04/12/GOesNq.jpg" alt="amov logo" align="right" height="60" />

# Prometheus - 自主无人机开源项目

**Prometheus**，在希腊神话中，是最具智慧的神明之一，希望本项目能为无人机研发工作带来无限的智慧与光明。

## 项目总览

Prometheus是一套开源的**自主无人机软件平台**，为无人机的智能与自主飞行提供**全套解决方案**。本项目基于**PX4**开源飞控固件，旨在为PX4开发者配套成熟可用的**机载电脑端**程序，提供更加简洁快速的开发体验。目前已集成**建图**、**定位**、**规划**、**控制**及**目标检测**等模块，并配套有Gazebo仿真测试代码。

 - **安装及使用：** Prometheus开发者手册
 - **配套硬件购买：** 阿木淘宝店 或 联系微信：****

配图:roadmap 及 项目总览图(说明结构和功能)

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
- **YOLO通用目标检测(暂缺)**

- **移动平台自主降落**

	<img width="400" src="https://s1.ax1x.com/2020/04/12/GOAZQJ.gif"/>

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
