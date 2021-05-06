# Note!
Our recently developed planner [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) is an evolution from EGO-Planner. 
It is more robust and safe, and therefore, is more recommended to use.
If you have only one drone, just set the `drone_id` to `0` in EGO-Swarm's launch files.
Of course, some topic names are changed from EGO-Planner, check it using `rqt_graph` and `rosnode info <package name>`.

# Quick Start within 3 Minutes 
Compiling tests passed on ubuntu **16.04, 18.04 and 20.04** with ros installed.
You can just execute the following commands one by one.
```
sudo apt-get install libarmadillo-dev
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
cd ego-planner
catkin_make
source devel/setup.bash
roslaunch ego_planner simple_run.launch
```
If your network to github is slow, We recommend you to try the gitee repository [https://gitee.com/iszhouxin/ego-planner](https://gitee.com/iszhouxin/ego-planner). They synchronize automatically.

If you find this work useful or interesting, please kindly give us a star :star:, thanks!:grinning:

# Acknowledgements
- The framework of this repository is based on [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) by Zhou Boyu who achieves impressive proformance on quaorotor local planning.

- The L-BFGS solver we use is from [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite). 
It is a C++ head-only single file, which is lightweight and easy to use.

- The map generated in simulation is from [mockamap](https://github.com/HKUST-Aerial-Robotics/mockamap) by William Wu.

- The hardware architecture is based on an open source implemation from [Teach-Repeat-Replan](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan).

# EGO-Planner 
EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors

**EGO-Planner** is a lightweight gradient-based local planner without ESDF construction, which significantly reduces computation time compared to some state-of-the-art methods <!--(EWOK and Fast-Planner)-->. The total planning time is only **around 1ms** and don't need to compute ESDF.

<p align = "center">
<img src="pictures/title.gif" width = "413" height = "232" border="5" />
<img src="pictures/comp.jpg" width = "413" height = "232" border="5" />
<img src="pictures/indoor.gif" width = "413" height = "232" border="5" />
<img src="pictures/outdoor.gif" width = "413" height = "232" border="5" />
</p>

**Video Links:** [YouTube](https://youtu.be/UKoaGW7t7Dk), [bilibili](https://www.bilibili.com/video/BV1VC4y1t7F4/) (for Mainland China)

## 1. Related Paper
EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors, Xin Zhou, Zhepei Wang, Chao Xu and Fei Gao (Accepted by RA-L). [Preprint](https://arxiv.org/abs/2008.08835) and [Early Access](https://ieeexplore.ieee.org/document/9309347).

## 2. Standard Compilation

**Requirements**: ubuntu 16.04, 18.04 or 20.04 with ros-desktop-full installation.

**Step 1**. Install [Armadillo](http://arma.sourceforge.net/), which is required by **uav_simulator**.
```
sudo apt-get install libarmadillo-dev
``` 

**Step 2**. Clone the code from github or gitee. This two repositories synchronize automaticly.

From github,
```
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
```

Or from gitee,
```
git clone https://gitee.com/iszhouxin/ego-planner.git
```

**Step 3**. Compile,
```
cd ego-planner
catkin_make -DCMAKE_BUILD_TYPE=Release
```

**Step 4**. Run.

In a terminal at the _ego-planner/_ folder, open the rviz for visuallization and interactions
```
source devel/setup.bash
roslaunch ego_planner rviz.launch
```

In another terminal at the _ego-planner/_, run the planner in simulation by
```
source devel/setup.bash
roslaunch ego_planner run_in_sim.launch
```

Then you can follow the gif below to control the drone.

<p align = "center">
<img src="pictures/sim_demo.gif" width = "640" height = "438" border="5" />
</p>

## 3. Using an IDE
We recommend using [vscode](https://code.visualstudio.com/), the project file has been included in the code you have cloned, which is the _.vscode_ folder.
This folder is **hidden** by default.
Follow the steps below to configure the IDE for auto code completion & jump.
It will take 3 minutes.

**Step 1**. Install C++ and CMake extentions in vscode.

**Step 2**. Re-compile the code using command
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
```
It will export a compile commands file, which can help vscode to determine the code architecture.

**Step 3**. Launch vscode and select the _ego-planner_ folder to open.
```
code ~/<......>/ego-planner/
```

Press **Ctrl+Shift+B** in vscode to compile the code. This command is defined in _.vscode/tasks.json_.
You can add customized arguments after **"args"**. The default is **"-DCMAKE_BUILD_TYPE=Release"**.

**Step 4**. Close and re-launch vscode, you will see the vscode has already understood the code architecture and can perform auto completion & jump.

 ## 4. Use GPU or Not
 Packages in this repo, **local_sensing** have GPU, CPU two different versions. By default, they are in CPU version for better compatibility. By changing
 
 ```
 set(ENABLE_CUDA false)
 ```
 
 in the _CMakeList.txt_ in **local_sensing** packages, to
 
 ```
 set(ENABLE_CUDA true)
 ```
 
CUDA will be turned-on to generate depth images as a real depth camera does. 

Please remember to also change the 'arch' and 'code' flags in the line of 
```
    set(CUDA_NVCC_FLAGS 
      -gencode arch=compute_61,code=sm_61;
    ) 
``` 
in _CMakeList.txt_, if you encounter compiling error due to different Nvidia graphics card you use. You can check the right code [here](https://github.com/tpruvot/ccminer/wiki/Compatibility).
 
Don't forget to re-compile the code!

**local_sensing** is the simulated sensors. If ```ENABLE_CUDA``` **true**, it mimics the depth measured by stereo cameras and renders a depth image by GPU. If ```ENABLE_CUDA``` **false**, it will publish pointclouds with no ray-casting. Our local mapping module automatically selects whether depth images or pointclouds as its input.

For installation of CUDA, please go to [CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit)

## 5. Utilize the Full Performance of CPU
The computation time of our planner is too short for the OS to increase CPU frequency, which makes the computation time tend to be longer and unstable.

Therefore, we recommend you to manually set the CPU frequency to the maximum.
Firstly, install a tool by
```
sudo apt install cpufrequtils
```
Then you can set the CPU frequency to the maximum allowed by
```
sudo cpufreq-set -g performance
```
More information can be found in [http://www.thinkwiki.org/wiki/How_to_use_cpufrequtils](http://www.thinkwiki.org/wiki/How_to_use_cpufrequtils).

Note that CPU frequency may still decrease due to high temperature in high load.

# Improved ROS-RealSense Driver

We modified the ros-relasense driver to enable the laser emitter strobe every other frame, allowing the device to output high quality depth images with the help of emitter, and along with binocular images free from laser interference.

<p align = "center">
<img src="pictures/realsense.PNG" width = "640" height = "158" border="5" />
</p>

This ros-driver is modified from [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) and is compatible with librealsense2 2.30.0.
Tests are performed on Intel RealSense D435 and D435i.

Parameter ```emitter_on_off``` is to turn on/off the added function.
Note that if this function is turned on, the output frame rate from the device will be reduced to half of the frame rate you set, since the device uses half of the stream for depth estimation and the other half as binocular grayscale outputs.
What's more, parameters ```depth_fps``` and ```infra_fps``` must be identical, and ```enable_emitter``` must be true as well under this setting.

##  Install

The driver of librealsense2 2.30.0 should be installed explicitly.
On a x86 CPU, this can be performed easily within 5 minutes.
Firstly, remove the currently installed driver by 
```
sudo apt remove librealsense2-utils
```
or manually remove the files if you have installed the librealsense from source.
Then, you can install the library of version 2.30.0 by
```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```
For ubuntu 16.04
```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
```
For ubuntu 18.04
```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
```
Then continue with
```
sudo apt-get install librealsense2-dkms
sudo apt install librealsense2=2.30.0-0~realsense0.1693
sudo apt install librealsense2-gl=2.30.0-0~realsense0.1693
sudo apt install librealsense2-utils=2.30.0-0~realsense0.1693
sudo apt install librealsense2-dev=2.30.0-0~realsense0.1693
sudo apt remove librealsense2-udev-rules
sudo apt install librealsense2-udev-rules=2.30.0-0~realsense0.1693
``` 
Here you can varify the installation by 
```
realsense-viewer
```

##  Run

If everything looks well, you can now compile the ros-realsense package named _modified_realsense2_camera.zip_ by ```catkin_make```, then run ros realsense node by 
```
roslaunch realsense_camera rs_camera.launch
```
Then you will receive depth stream along with binocular stream together at 30Hz by default.

<!--
# A Lightweight Quadrotor Simulator

The quadrotor simulator we use is inherited and modified from [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner). 
It is lightweight and super easy to use.
Only one topic is required to control the drone.
You can execute 
```
roslaunch so3_quadrotor_simulator simulator_example.launch 
```
to run a simple example in ego-planner/src/uav_simulator/so3/control/src/control_example.cpp.
If this simulator is helpful to you, plaease kindly give a star to [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) as well.-->

# Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

# Maintaince
We are still working on extending the proposed system and improving code reliability. 

For any technical issues, please contact Xin Zhou (iszhouxin@zju.edu.cn) or Fei GAO (fgaoaa@zju.edu.cn).

For commercial inquiries, please contact Fei GAO (fgaoaa@zju.edu.cn).
