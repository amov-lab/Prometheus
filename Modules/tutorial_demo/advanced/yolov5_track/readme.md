# yolov5识别+siamrpn点击跟踪
![yolov5.gif](https://qiniu.md.amovlab.com/img/m/202206/20220606/1132215673307727086911488.gif)

# 必要阅读
1. 仿真环境遥控器操作
2. 请根据[Modules/object_detection/readme.md](../../../object_detection/readme.md)进行相应到依赖安装(需要配置tensorrt环境,tensorrt环境配置难度较大)
# 推荐阅读
1. [yolov5算法概述](https://github.com/amov-lab/Prometheus/wiki/Prometheus%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E7%AE%97%E6%B3%95-%E9%80%9A%E7%94%A8%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B)
2. [跟踪+检测](https://github.com/amov-lab/Prometheus/wiki/Prometheus%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E7%AE%97%E6%B3%95-%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B+%E7%82%B9%E5%87%BB%E8%B7%9F%E8%B8%AA)
3. [siamrpn目标跟踪](Modules/tutorial_demo/advanced/siamrpn_track/readme.md)

# 使用

1. 安装`Modules/object_detection_yolov5tensorrt`，运行`Scripts/installation/object_detection/install_detection_yolov5tensorrt.sh`。__注意载Prometheus根目录下运行__
2. 确保步骤1成功完成
3. 打开终端进入Prometheus根目录，执行以下命令
```bash
roslaunch prometheus_demo yolov5_track_all.launch
# 另外开一个窗口运行进入Modules/object_detection_yolov5tensorrt路径
# 注意在prometheus_python3环境下执行领命, 比如我到就是：
# (prometheus_python3) onx@onx:~$ python3 yolov5_tensorrt_client.py
python3 yolov5_trt_ros.py  --image_topic /prometheus/sensor/monocular_front/image_raw
```
4. 等待程序全部启动完成，2个终端窗口，1个图像窗口
5. 点击目标
4. 遥控器解锁无人机，并切换到`RC_POS_CONTROL`模式，等待无人机起飞并保持悬停
5. 遥控器切换到`COMMAND_CONTROL`，无人机靠近目标，并在距离目标一定距离后保存悬停

# 程序核心逻辑
- `Modules/object_detection_yolov5tensorrt/yolov5_trt_ros.py` 加载yolov5的tensorrt模型和siamrpn模型。由于仿真环境，本程序的yolov5使用的x86分支版本，如果想要在NX等ARM的设备上使用切换分支。`yolov5_trt_ros.py`支持从摄像头获取图像和从ros话题中获取图像，简单测试使用`python3 yolov5_trt_ros.py --no_tcp`。

- `yolov5_trt_ros.py`首先使用yolov5进行目标识别，在鼠标点击相应目标框后，程序将目标宽中到图像截取放入siamrpn进行目标跟踪(识别和跟踪的区别：识别，同一时刻，区分不同类别。跟踪，当前时刻与上时刻是否同一个物体，但不一定知道类别)。对于检测&跟踪的信息通过TCP协议发送，信息主要包括，类别、所在图像位置、当前所出状态(检测还是跟踪)。

`yolov5_tensorrt_client.py`接收`yolov5_trt_ros.py`发送的数据，转换为`Modules/common/prometheus_msgs/msg/DetectionInfo.ms`ros消息发送。`Modules/tutorial_demo/advanced/siamrpn_track/src/siamrpn_track.cpp`接受目标信息控制无人机接近目标。

