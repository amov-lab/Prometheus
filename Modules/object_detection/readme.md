**在RTX 3060、Ubuntu18.04、X86主机环境中通过测试，其他环境(如ARM上)可能会遇到问题**

# Dependencies

- python3 部分程序依赖于python3
- pytorch

## 配置 Melodic cv_bridge 支持 python3
> https://blog.actorsfit.com/a?ID=01750-a3d568ec-ce66-4961-9573-0f9bba2561f8

```bash
sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge
mkdir -p catkin_workspace/src
cd catkin_workspace
# 注意检查路径是否存在
catkin config -DPYTHON_EXECUTABLE =/usr/bin/python3 -DPYTHON_INCLUDE_DIR =/usr/include/python3.6m -DPYTHON_LIBRARY =/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install
git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv
```

查看当前系统cv_bridge到版本，编译相应到版本
```bash
apt-cache show ros-melodic-cv-bridge |  grep Version
# Version: 1.13.0-0bionic.20220127.152918

cd src/vision_opencv/
git checkout 1.13.0
cd  ../../
catkin build cv_bridge
source install/setup.bash --extend
```

加入到`.bashrc`中，注意加在上一条 source 之后，否则可能会导致包覆盖，而找不到某些包

**example**:
```bash
....
source /opt/ros/melodic/setup.bash
source /home/onx/prometheus_mavros/devel/setup.bash
source /home/onx/Code/Prometheus/devel/setup.bash
source ~/catkin_workspace/install/setup.bash --extend
## Promehteus
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/onx/Code/Prometheus/devel/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/onx/Code/Prometheus/Simulator/gazebo_simulator/gazebo_models/uav_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/onx/Code/Prometheus/Simulator/gazebo_simulator/gazebo_models/ugv_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/onx/Code/Prometheus/Simulator/gazebo_simulator/gazebo_models/sensor_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/onx/Code/Prometheus/Simulator/gazebo_simulator/gazebo_models/scene_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/onx/Code/Prometheus/Simulator/gazebo_simulator/gazebo_models/texture

## Promehteus_PX4
source /home/onx/Code/prometheus_px4/Tools/setup_gazebo.bash /home/onx/Code/prometheus_px4 /home/onx/Code/prometheus_px4/build/amovlab_sitl_default >> /dev/null
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/onx/Code/prometheus_px4
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/onx/Code/prometheus_px4/Tools/sitl_gazebo
....
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

# 代码说明

- 加载不同的相机内参参数，进行二维码检测估计
```
aruco_det_imx477_960x540.launch
aruco_det_webcam_640x480.launch
aruco_det.launch
```
- 启动相关相机并转化为ROS话题发布
```
mipi_cam_720p.launch
mipi_cam_imx477_960x540.launch
mipi_cam_imx477_1080p.launch
```
- yolo(darnet版)目标检测，加载模型，从图像话题中获取视频流，发布检测信息
```
ms_coco_det.launch # 加载coco数据集训练的模型
uav_det.launch #
```
- 从不同的图像话题获取视频流，进行框选跟踪
```
tracker_kcf_gazebo.launch # KCF跟踪
tracker_kcf.launch # KCF跟踪
tracker_siamrpn.launch # siamrpn跟踪
```
- 将本地视频转换为图像话题
```