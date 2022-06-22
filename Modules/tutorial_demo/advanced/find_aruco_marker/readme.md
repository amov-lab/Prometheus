# 多二维码识别自主降落
![多二维码.gif](https://qiniu.md.amovlab.com/img/m/202206/20220606/1130165150895711620857856.gif)
# 必要阅读
1. 仿真环境遥控器操作

# 推荐阅读
2. [二维码检测原理概述](https://github.com/amov-lab/Prometheus/wiki/Prometheus%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E7%AE%97%E6%B3%95-%E4%BA%8C%E7%BB%B4%E7%A0%81%E6%A3%80%E6%B5%8B)

# 使用
_多个二维码检测比较吃CPU资源，可能会导致卡顿_
1. 打开终端进入Prometheus根目录，执行以下命令
```bash
./Scripts/simulation/tutorial_demo/find_aruco_marker.sh
```
1. 等待程序全部启动完成
3. 切换rqt_image_view的图像话题到`/uav1/detection/image`，查看二维码检测的可视化(无人机在没有起飞时，摄像头朝下画面可能为白色)
![image.png](https://qiniu.md.amovlab.com/img/m/202206/20220616/1807212716561216578158592.png)
4. 输入二维码id，目前world中有id为1~20二维码，比如这里输入17
![image.png](https://qiniu.md.amovlab.com/img/m/202206/20220616/1809113174931085385433088.png)
4. 遥控器解锁无人机，并切换到`RC_POS_CONTROL`模式，等待无人机起飞并保持悬停
5. 遥控器切换到`COMMAND_CONTROL`，无人机绕圆飞行，并且在绕圆到过程中进行二维码识别，如果识别到给定id的二维码则终端绕圆飞行，前往对应二维码进行自主降落

# 程序核心逻辑
`Modules/object_detection/cpp_nodes/aruco_det.cpp`有3种运行模式`run_state`
- 0: 二维码检测
- 1: 使用二维码标定板，以二维码标定版的中心作为新的世界坐标系原点，完成标定后自动进入模式2
- 2: 发布全局map先对于相机的位姿，发布检测到每个二维码位姿
- 3: 通过平均值方法，使用无人机信息估计每个二维码的在世界坐标系位姿。

# TODO:
模式1，2应用场景：标定世界坐标位姿，然后检测其他二维码的位姿，就可以通过标定了的世界坐标系位姿，计算出其他坐标系相对于世界坐标系的位姿，_而不在是相机坐标系的位姿_。

本程序只使用`Modules/object_detection/cpp_nodes/aruco_det.cpp`中`run_state==0`模式。发发布所有二维码在相机坐标系位置。`Modules/tutorial_demo/advanced/find_aruco_marker/src/main.cpp`接收二维码信息，转化为机体系坐标，在机体坐标系控制无人机到达二维码上方悬停。



