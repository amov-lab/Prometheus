# Prometheus_gazebo

## 准备工作
 - PX4固件需已能够编译，并能运行其自带的Gazebo仿真
 - Mavros功能包
 - Prometheus项目中其他相应的功能包均已编译通过

## 编译

```
cd Prometheus
./compile_gazebo_simulator.sh
```

## 环境变量设置

在bash文件中手动添加：

```
sudo gedit ~/.bashrc 
```

复制粘贴如下内容至文件末尾：（针对PX4 1.8.2版本）

```
source /home/fly-vision/Prometheus/devel/setup.bash
export GAZEBO_MODEL_PATH=:${your path}/Prometheus/Simulator/gazebo_simulator/models:~/gazebo_models
source ${your px4 path}/Firmware/Tools/setup_gazebo.bash ${your px4 path}/Firmware ${your px4 path}/Firmware/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware/Tools/sitl_gazebo
```

其中${your path}为Prometheus项目路径，${your px4 path}为安装px4的路径。

对于PX4 1.9.2版本
```
source /home/fly-vision/Prometheus/devel/setup.bash
export GAZEBO_MODEL_PATH=:${your path}/Prometheus/Simulator/gazebo_simulation/src/iris_gazebo/models:~/gazebo_models
source ${your px4 path}/Firmware/Tools/setup_gazebo.bash ${your px4 path}/Firmware ${your px4 path}/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware/Tools/sitl_gazebo
```

## 运行

运行如下命令测试是否Gazebo仿真是否正确配置
```
roslaunch prometheus_gazebo sitl_test.launch
```
可打开QGC，解锁并起飞，观察飞机是否能够起飞。

### 位置环控制器仿真

运行如下命令
```
roslaunch prometheus_gazebo sitl_control.launch
```

### 自主降落仿真

```
roslaunch iris_gazebo landing_with_qrcode.launch
```




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

## 二维码检测仿真

通过运行如下launch文件启动二维码检测仿真：

```
roslaunch iris_gazebo landing_with_qrcode.launch
```

该launch文件中所使用到的model都在项目文件夹中，所以需要在bashrc文件中将模型路径加入到环境变量中，即复制以下内容到文件末尾。

```
export GAZEBO_MODEL_PATH=:${your path}/Prometheus/Simulator/gazebo_simulation/src/iris_description/models
```

其中后，会出现安装了方向向下的相机的iris无人机模型，以及做圆周运动的二维码模型。