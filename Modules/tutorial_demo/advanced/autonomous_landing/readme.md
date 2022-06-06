# 二维码自主降落
![二维码自主降落.gif](https://qiniu.md.amovlab.com/img/m/202206/20220606/1126474273967363998515200.gif)
# 使用
```bash
roslaunch prometheus_demo autonomous_landing.launch
```

运行`rqt_image_view`，切换话题到`/prometheus/camera/rgb/image_landpad_det`可显示降落点可视化。

# 描述

`Modules/object_detection/cpp_nodes/markers_landpad_det.cpp`接收图像信心，检测二维码，发布降落点在相机坐标系下的位置，消息为`Modules/common/prometheus_msgs/msg/DetectionInfo.msg`。`Modules/tutorial_demo/advanced/autonomous_landing/src/autonomous_landing.cpp`接收消息并结合无人机姿态、位置计算出目标在**机体惯性坐标系**下的位置。有了目标在机体惯性系坐标后，就可以根据各个轴的距离误差动态控制无人机速度接近降落点。