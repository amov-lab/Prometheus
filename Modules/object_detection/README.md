# vision_ws
Vision code for P300 quadrotor platform

方向定义： 目标位置 [机体系下：前方x为正，右方y为正，下方z为正]

标志位：   orientation.w 用作标志位 1代表识别到目标 0代表丢失目标

发布话题适用于椭圆、二维码、yolo等视觉算法

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
将得到的参数写入
```
Prometheus/Modules/object_detection/config/camera_param.yaml
```
修改每个算法的launch文件，指向以上参数文件camera_param.yaml
```
vision_ws/src/vision_ws/aruco_det_ros/launch/aruco_det.launch
vision_ws/src/vision_ws/darknet_ros/darknet_ros/launch/darknet_ros.launch
vision_ws/src/vision_ws/ellipse_det_ros/launch/ellipse_det.launch
vision_ws/src/vision_ws/ellipse_det_ros/launch/ellipse_det_wt.launch
vision_ws/src/vision_ws/landpad_det_ros/launch/landpad_det.launch
```

## 运行
1.降落板检测
```
roslaunch prometheus_detection aruco_det.launch # 二维码检测
roslaunch prometheus_detection landpad_det.launch # 降落板检测
```
2.椭圆检测
```
roslaunch prometheus_detection ellipse_det.launch # 所有的椭圆
roslaunch prometheus_detection ellipse_det_wt.launch # 带训练的指定椭圆
```
3.YOLO检测
```
roslaunch prometheus_detection darknet_ros.launch
```
4.数字检测
```
roslaunch prometheus_detection num_det.launch
```
