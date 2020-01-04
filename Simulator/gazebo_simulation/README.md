# Gazebo仿真

## Gazebo

仿真包含深度相机、双目相机及iris无人机模型，运行环境为gazebo9和ROS，使用如下命令进行安装：

For Ubuntu 16.04:

```
sudo apt install ros-kinetic-gazebo9*
```

For Ubuntu 18.04:

```
sudo apt install ros-melodic-gazebo9*
```

## px4 Firmware

下载px4源码并切换至v1.9.2的固件

```
git clone https://github.com/PX4/Firmware
```

更新submodule，切换固件并编译

```
cd Firmware
git submodule update --init --recursive
git checkout v1.9.2
make distclean
make px4_sitl_default gazebo
```

下载gazebo的模型包，在home目录下创建gazebo_models文件夹

```
yourname@ubuntu:~$ mkdir gazebo_models
```

下载gazebo模型包 https://bitbucket.org/osrf/gazebo_models/downloads/

把gazebo模型包解压出来的所有模型文件剪切至**gazebo_models**文件夹

## 运行iris仿真

在Prometheus/Simulator/gazebo_simulation文件夹下进行编译：

```
catkin_make
```

编译成功后运行source_environment.sh文件，将Firmware路径，gazebo的模型路径等加入环境变量

```
source source_enviroment.sh
```

或者在bash文件中手动添加：

```
sudo gedit ~/.bashrc 
```

复制粘贴如下内容至文件末尾：

```
source  ${your path}/Prometheus/Simulator/gazebo_simulation/devel/setup.bash
export GAZEBO_MODEL_PATH=:${your path}/Prometheus/Simulator/gazebo_simulation/src/iris_description/models:~/gazebo_models
source ${your px4 path}/Firmware/Tools/setup_gazebo.bash ${your px4 path}/Firmware ${your px4 path}/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware/Tools/sitl_gazebo
```

其中${your path}为Prometheus项目路径，${your px4 path}为安装px4的路径。

打开终端，运行如下命令，启动iris的仿真：

```
roslaunch iris_gazebo iris_with_realsense.launch
```

 keyboard control的节点也会一同启动，但程序不能单独使用,需要在QGC中配置遥控器通道相关参数，具体参数如下：

```
RC2_TRIM = 1000us
COM_FLTMODE1 = Position
RC_CHAN_CNT = 8
RC_MAP_FLTMODE = Channel 5
RC_MAP_PITCH = Channel 3
RC_MAP_ROLL= Channel 1
RC_MAP_THROTTLE = Channel 2
RC_MAP_YAW = Channel 4
```

## Octomap地图

首先使用如下命令安装Octomap:

For Ubuntu 16.04:

```
sudo apt-get install ros-kinetic-octomap-ros #安装octomap
sudo apt-get install ros-kinetic-octomap-msgs
sudo apt-get install ros-kinetic-octomap-server
```

然后再安装octomap在rviz中的插件

```
sudo apt-get install ros-kinetic-octomap-rviz-plugins
```

安装完插件后，再运行rviz时会出现octomap的模组。

通过如下命令运行带有realsense相机的iris无人机仿真，并运行rtabmap，其作用是接受RGBD相机发布的深度图及里程计发布的相机位姿，并以此为基础发布的各种地图topic，包括pointcloud, grid map和octomap。

```
roslaunch iris_gazebo iris_with_realsense.launch
roslaunch iris_gazebo rtabmap_depthCam_mapping.launch
```

此时会有octomap的topic发布，运行rviz后选择add添加‘ColorOccupancyGrid’并选择octomap的topic，便会看到实时显示的octomap。