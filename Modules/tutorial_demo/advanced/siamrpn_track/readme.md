# 框选跟踪
![siamrpn.gif](https://qiniu.md.amovlab.com/img/m/202206/20220606/1130575322852171291459584.gif)

# 必要阅读
1. 仿真环境遥控器操作
2. [请根据`Modules/object_detection/readme.md`](../../../object_detection/readme.md)进行相应到依赖安装(不要配置tensorrt环境)

# 推荐阅读
1. [siamrpn算法概述](https://github.com/amov-lab/Prometheus/wiki/Prometheus%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E7%AE%97%E6%B3%95-%E7%9B%AE%E6%A0%87%E6%A1%86%E9%80%89%E8%B7%9F%E8%B8%AA)

# 使用

1. 打开终端进入Prometheus根目录，执行以下命令
```bash
roslaunch prometheus_demo siamrpn_track_all.launch
```
2. 等待程序全部启动完成，2个终端窗口，1个图像窗口
3. 框选目标
4. 遥控器解锁无人机，并切换到`RC_POS_CONTROL`模式，等待无人机起飞并保持悬停
5. 遥控器切换到`COMMAND_CONTROL`，无人机靠近目标，并在距离目标一定距离后保存悬停

# 程序核心逻辑
`Modules/object_detection/py_nodes/siamrpn_tracker/siam_rpn.py`加载siamrpn跟踪模型接收图像话题，发布`Modules/common/prometheus_msgs/msg/DetectionInfo.msg`消息，消息包含对目标位置，姿态，视线角的估计值。`Modules/tutorial_demo/advanced/siamrpn_track/src/siamrpn_track.cpp`接受消息，控制无人机跟踪宽选到目标。