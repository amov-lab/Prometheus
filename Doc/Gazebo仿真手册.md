#### 无人机仿真手册
- 切换Prometheus_PX4分支

- 环境变量配置

```shell
## Promehteus
source /home/sysu/Prometheus/devel/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/sysu/Prometheus/devel/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/sysu/Prometheus/Simulator/gazebo_simulator/gazebo_models/uav_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/sysu/Prometheus/Simulator/gazebo_simulator/gazebo_models/ugv_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/sysu/Prometheus/Simulator/gazebo_simulator/gazebo_models/sensor_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/sysu/Prometheus/Simulator/gazebo_simulator/gazebo_models/scene_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/sysu/Prometheus/Simulator/gazebo_simulator/gazebo_models/texture

## Promehteus_PX4
source /home/sysu/prometheus_px4/Tools/setup_gazebo.bash /home/sysu/prometheus_px4 /home/sysu/prometheus_px4/build/amovlab_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/sysu/prometheus_px4
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/sysu/prometheus_px4/Tools/sitl_gazebo
```

- 启动仿真(GPS定位模式)
roslaunch prometheus_gazebo sitl_outdoor.launch
roslaunch prometheus_uav_control uav_control_main_outdoor.launch 

- 启动仿真(模拟室内定位模式)
roslaunch prometheus_gazebo sitl_indoor.launch
roslaunch prometheus_uav_control uav_control_main_indoor.launch 

- 启动仿真(GPS定位模式) - 无遥控器
roslaunch prometheus_gazebo sitl_outdoor.launch
roslaunch prometheus_uav_control uav_control_main_no_rc.launch 