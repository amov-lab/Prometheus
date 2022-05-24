# Dependencies

- python3: 此demo不支持python2
- pytorch

# 配置 Melodic cv_bridge 支持 python3
> https://blog.actorsfit.com/a?ID=01750-a3d568ec-ce66-4961-9573-0f9bba2561f8

```bash
sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge
mkdir -p catkin_workspace/src
cd catkin_workspace
# 注意检测路径是否存在
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

# 使用

```bash
roslaunch prometheus_demo siamrpn_track.launch
```