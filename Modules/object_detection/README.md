# Prometheus-object-detection
Vision code for P300 quadrotor platform

方向定义： 目标位置 [相机系下：右方x为正，下方y为正，前方z为正]

默认订阅图像话题： /prometheus/camera/rgb/image_raw  (可使用rosrun prometheus_detection web_cam运行相机节点)

默认发布话题：  /prometheus/target (话题格式请参考Prometheus/Modules/msgs/msg/DetectionInfo.msg)

发布话题适用于椭圆、二维码、方框中的手写数字、kcf目标跟踪、yolo等视觉算法

## 安装
```
cd ~
git clone https://github.com/amov-lab/Prometheus.git
./complie_detection.sh
```

## 相机标定
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0245 image:=/usb_cam/image_raw camera:=/usb_cam
```
size为标点板尺寸，square为每个方格宽度(m)，image:=相机话题
将得到的参数写入如下文件(有关目标尺度的预定义也在这个文件中)
```
Prometheus/Modules/object_detection/config/camera_param.yaml
```

## 运行
```
roscore
```
1.降落板检测(降落板的具体描述见附录1.1)
```
rosrun prometheus_detection landpad_det
```
2.单个二维码检测(二维码示例见附录1.2)
```
rosrun prometheus_detection aruco_det
```
3.椭圆检测(可以通过训练的方式检测中心有特定图案的椭圆，具体方式见附录1.3)
```
rosrun prometheus_detection ellipse_det # 所有的椭圆
rosrun prometheus_detection ellipse_det -wt # 带训练的指定椭圆
```
4.目标跟踪
```
rosrun prometheus_detection tracker_kcf
```
3.YOLO检测
```
roslaunch prometheus_detection darknet_ros.launch
```
4.数字检测
```
roslaunch prometheus_detection num_det.launch
```

## 附录
### 1.1 降落板具体描述
