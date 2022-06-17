# 二维码自主降落
![二维码自主降落.gif](https://qiniu.md.amovlab.com/img/m/202206/20220606/1126474273967363998515200.gif)
# 必要阅读
1. 仿真环境遥控器操作

# 推荐阅读
2. [二维码检测概述](https://github.com/amov-lab/Prometheus/wiki/Prometheus%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E7%AE%97%E6%B3%95-%E4%BA%8C%E7%BB%B4%E7%A0%81%E6%A3%80%E6%B5%8B)

# 使用
1. 打开终端进入Prometheus根目录，执行以下命令
```bash
./Scripts/simulation/tutorial_demo/autonomous_landing.sh
```
2. 等待程序全部启动完成
3. 输入想要到达的二维码id（1～20），进行降落，比如输入: 3
![image.png](https://qiniu.md.amovlab.com/img/m/202206/20220616/164353157471908877467648.png)
4. 切换rqt_image_view的图像话题到`/uav1/prometheus/camera/rgb/image_landpad_det`，查看二维码检测的可视化(无人机在没有起飞时，摄像头朝下画面可能为白色)
![](https://qiniu.md.amovlab.com/img/m/202206/20220616/1515593517198422039953408.png)
4. 遥控器解锁无人机，并切换到`RC_POS_CONTROL`模式，等待无人机起飞并保持悬停
5. 遥控器切换到`COMMAND_CONTROL`，无人机飞行到一定高度识别二维码，执行降落程序，等待无人机完成降落

# 程序核心逻辑

`Modules/object_detection/cpp_nodes/markers_landpad_det.cpp`接收图像信心，检测二维码，发布降落点在相机坐标系下的位置，消息为`Modules/common/prometheus_msgs/msg/DetectionInfo.msg`。`Modules/tutorial_demo/advanced/autonomous_landing/src/autonomous_landing.cpp`接收消息并结合无人机姿态、位置计算出目标在**机体惯性坐标系**下的位置。有了目标在机体惯性系坐标后，就可以根据各个轴的距离误差动态控制无人机速度接近降落点。