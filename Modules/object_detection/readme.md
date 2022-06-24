**在RTX 3060、Ubuntu18.04、X86主机环境中通过测试，其他环境(如ARM上)可能会遇到问题**

# Dependencies

- python3 部分程序依赖于python3
- pytorch

## 配置 Melodic cv_bridge 支持 python3
```bash
sudo apt install ros-melodic-cv-bridge-python3
```
在每一个python3的脚本中，在文件头都要添加`#!/usr/bin/env python3`以及`import sys, os; sys.path.insert(0,'/opt/ros/' + os.environ['ROS_DISTRO'] + '/lib/python3/dist-packages/')`

例如在SiamRPN跟踪中：
```py
import sys
import os
path = sys.path[0]
path = path + '/../../src/siam_rpn_lib/'
print(path)
sys.path.append(path)
sys.path.append("/../../../../devel/lib/python2.7/dist-packages")
sys.path.insert(0,'/opt/ros/' + os.environ['ROS_DISTRO'] + '/lib/python3/dist-packages/')
import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
```

## 安装Python3相关环境

1. [安装并配置conda](https://docs.conda.io/en/latest/miniconda.html)。_conda详细安装教程较多，自行搜索_

2. 配置安装所需环境，进入object_detection目录`cd Modules/object_detection`, 执行：
```bash
conda env create -f conda_env.yaml
```
3. 等待上一命令执行完成后，conda会创建一个`prometheus_python3`名字的python虚拟环境。激活环境`conda activate prometheus_python3`.

4. 验证环境是否安装正确。执行以下命令，如果返回的**不是**`/usr/bin/python3`就说明正确安装。比如我这里返回到是`/home/onx/.conda/envs/prometheus_python3/bin/python3`
```bash
conda activate prometheus_python3
which python3
```

5. 加入`.bashrc`启动时自动激活`prometheus_python3`环境
```bash
echo "conda activate prometheus_python3" >> ~/.bashrc
```

# 使用案例
## 相机标定
创建一个标定板，运行后会在当前目录创建一个名为`board_charuco_d6.png`的图片，使用A4进行打印
```bash
./Modules/object_detection/shell/create_board_charuco.sh
```
运行标定程序，程序会读取相机id为0的画面，根据画面提示，按`c`键添加标定图像(建议添加20张以上)，然后按`esc`键程序开始根据获取到图像进行相机内参计算(画面会卡主，此时正在计算)，等待计算完成
```bash
./Modules/object_detection/shell/calibrate_camera_charuco_cam0.sh
```
将生成的文件重命名为`calib_webcam_640x480.yaml`替换掉`Modules/object_detection/shell/calib_webcam_640x480.yaml`中的文件

## 二维码检测
测试二维码检测`aruco_det.cpp`
```bash
# 默认使用0号摄像头
roslaunch prometheus_detection web_cam0.launch
# 启动二维码检测
roslaunch prometheus_detection aruco_det.launch
```

## yolov3目标检测
_注意配置安装Nvidia驱动，CUDA否则帧率只有1帧率_
```bash
# 默认使用0号摄像头
roslaunch prometheus_detection web_cam0.launch
roslaunch prometheus_detection ms_coco_det.launch
```
## KCF跟踪
```bash
# 默认使用0号摄像头
roslaunch prometheus_detection web_cam0.launch
roslaunch prometheus_detection tracker_kcf.launch
```
## SiamRPN跟踪
```bash
# 默认使用0号摄像头
roslaunch prometheus_detection web_cam0.launch
roslaunch prometheus_detection tracker_siamrpn.launch
```

## 其他
启动minip相机
```bash
roslaunch prometheus_detection mipi_cam_720p.launch
```
将视频文件转为ROS图像话题，修改`input_video_dir`路径为视频文件地址
```bash
roslaunch prometheus_detection video_replay.launch
```
yolov5的tensorrt加速，目标跟踪。需要另外其他内容参考[高级demo到yolov5的使用部分](Modules/tutorial_demo/advanced/yolov5_track/readme.md)
```bash
roslaunch prometheus_detection yolov5_nvidia_tensorrt.launch
```