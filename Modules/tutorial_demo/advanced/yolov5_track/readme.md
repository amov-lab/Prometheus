
# [请按照`Modules/object_detection/readme.md`](../../../object_detection/readme.md)进行相应到依赖安装
# RUN

1. 安装`Modules/object_detection_yolov5tensorrt`，运行`Scripts/object_detection/install_detection_yolov5tensorrt.sh`。__注意载Prometheus根目录下运行__
2. 启动
```bash
roslaunch prometheus_demo yolov5_track.launch
# 另外开一个窗口运行进入Modules/object_detection_yolov5tensorrt路径
# 注意在prometheus_python3环境下执行领命, 比如我到就是： (prometheus_python3) onx@onx:~$ python3 yolov5_tensorrt_client.py
python3 yolov5_trt_ros.py  --image_topic /prometheus/sensor/monocular_front/image_raw
```