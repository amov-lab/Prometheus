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

Gazebo仿真环境和功能写成单独的launch启动脚本
- Gazebo仿真环境启动脚本负责启动gazebo,生成无人机及相关模型,启动PX4 sitl,mavros
  - 此时所有无人机能够输出mavros状态,并可以接收mavros指令 
- 功能脚本负责启动相应功能,如单机控制脚本uav_control_main_outdoor.launch,多机控制脚本xxx.launch


- 启动仿真(GPS定位模式)
roslaunch prometheus_gazebo sitl_outdoor.launch
roslaunch prometheus_uav_control uav_control_main_outdoor.launch 

- 启动仿真(模拟室内定位模式)
roslaunch prometheus_gazebo sitl_indoor.launch
roslaunch prometheus_uav_control uav_control_main_indoor.launch 

- 启动仿真(GPS定位模式) - 无遥控器
roslaunch prometheus_gazebo sitl_outdoor.launch
roslaunch prometheus_uav_control uav_control_main_no_rc.launch 


- 启动多机(室内模式,4机) - 遥控器统一控制
roslaunch prometheus_gazebo sitl_indoor_4uav.launch
roslaunch prometheus_uav_control uav_control_main_indoor_4uav.launch