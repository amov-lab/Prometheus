<img src="https://s1.ax1x.com/2020/04/12/GOesNq.jpg" alt="amov logo" align="right" height="60" />

# Prometheus - 自主无人机开源项目

**Prometheus**，在希腊神话中，是最具智慧的神明之一，希望本项目能为无人机研发工作带来无限的智慧与光明。

## 项目总览

Prometheus是一套开源的**自主无人机软件平台**，为无人机的智能与自主飞行提供**全套解决方案**。本项目基于**PX4**开源飞控固件，旨在为PX4开发者配套成熟可用的**机载电脑端**程序，提供更加简洁快速的开发体验。目前已集成**建图**、**定位**、**规划**、**控制**及**目标检测**等模块，并配套有Gazebo仿真测试代码。

 - **安装及使用：** [Prometheus WIKI](https://github.com/amov-lab/Prometheus/wiki)

 - **配套课程：**    Prometheus自主无人机二次开发课程，本课程偏重本项目中的基础知识和操作实践，适合本项目入门者，购买课程后会定期安排线上答疑活动（语音交流、方便透彻！） [课程报名](https://bbs.amovlab.com/plugin.php?id=zhanmishu_video:video&mod=video&cid=43)
  
 - **配套硬件：**    即将推出，敬请期待！ 请关注 [阿木实验室淘宝店](https://shop142114972.taobao.com/?spm=a230r.7195193.1997079397.2.67d03d8dJQgFRW)　或　[阿木实验室京东旗舰店](https://mall.jd.com/index-10260560.html?from=pc)
 
 - **配套地面站：**    即将推出，敬请期待！ 
 
 - **开发及维护团队：** [开发成员介绍](https://github.com/amov-lab/Prometheus/wiki/开发成员介绍) 

    <img src="https://s1.ax1x.com/2020/08/13/dSEJhR.png" alt="amov qrcode" align="right" height="200" />

 - **开源项目，维护不易，还烦请点一个star收藏，谢谢支持！**
 
 - 关于项目有任何问题，请在本项目issue提问，或者添加微信jiayue199506（备注消息：Prometheus）进入Prometheus自主无人机交流群
  
 - 想了解更多资讯信息或者寻求项目合作、无人机软硬件定制服务，请扫描右方二维码并关注微信公众号 **阿木实验室**
 
## Prometheus校园赞助计划

只要使用Prometheus项目进行自主无人机开发，并发表论文，即可获得阿木奖学金，详情请戳：

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
	
	<img width="400" src="https://s1.ax1x.com/2020/08/11/aLawvj.gif"/>

- **无人机＋无人车协同控制**
<img width="400" src="https://s1.ax1x.com/2020/10/13/0fAJN6.gif"/>

## 版权申明

 - 本项目受 BSD 3-Clause 协议保护。点击 [LICENSE](https://github.com/amov-lab/Prometheus/blob/master/LICENSE)了解更多
 - 本项目仅限个人使用，请勿用于商业用途。
 - 如利用本项目进行营利活动，阿木实验室将追究侵权行为。
