# [请根据`Modules/object_detection/readme.md`](../../../object_detection/readme.md)进行相应到依赖安装

# 使用

```bash
roslaunch prometheus_demo siamrpn_track.launch
```
框选目标

# 描述
[原理描述](https://github.com/amov-lab/Prometheus/wiki/Prometheus%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E7%AE%97%E6%B3%95-%E7%9B%AE%E6%A0%87%E6%A1%86%E9%80%89%E8%B7%9F%E8%B8%AA)

## 逻辑描述
`Modules/object_detection/py_nodes/siamrpn_tracker/siam_rpn.py`加载siamrpn跟踪模型接收图像话题，发布`Modules/common/prometheus_msgs/msg/DetectionInfo.msg`消息，消息包含对目标位置，姿态，视线角的**估计值**。`Modules/tutorial_demo/advanced/siamrpn_track/src/siamrpn_track.cpp`接受消息，控制无人机跟踪宽选到目标.