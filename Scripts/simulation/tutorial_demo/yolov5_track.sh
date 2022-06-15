#!/bin/bash
# 脚本描述： 使用yolov5对目标进行识别，然后点击识别框使用siamrpn进行目标跟踪(需要GPU，且配置按文档配置好环境)
# 相关文档： Modules/tutorial_demo/advanced/yolov5_track/readme.md

roslaunch prometheus_demo yolov5_track.launch
python3 yolov5_trt_ros.py  --image_topic /prometheus/sensor/monocular_front/image_raw