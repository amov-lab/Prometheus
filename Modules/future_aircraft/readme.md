# future_aircraft 功能包使用介绍

## 一、介绍及使用

- 本功能包为第二届大学生未来飞行器挑战赛的实践类仿真demo
  - 设置比赛区域（10 * 20 * 6 m，暂定）场地内随机模拟圆形标靶 3 个；其中， 2 个随
    机固定在场地位置， 1 个固定于移动的无人车上面，无人车以≤ 1m/s 的速度在
    8 字形轮廓进行移动。  
  - 无人机由指定的位置一键起飞后，立即转换为自动模式，开始通过机载传感器
    自主搜索这些目标标靶，并向目标标靶投掷模拟子弹；完成所有的投掷任务后，
    自主回到起飞点降落。以投中目标的数量和完成时间来综合计分。  
  - 模拟子弹选用与标靶粘接的子弹，以减少因弹性或是风力影响。  
  - 主办方在赛场内提供 UWB 基站信号覆盖，参赛队伍可以自主选择目标识别和
    定位方式。  

![image.png](https://qiniu.md.amovlab.com/img/m/202207/20220722/1153184932903574280503296.png)

- 编译必要的文件（在Prometheus跟目录下运行）

  - ```
    ./Modules/future_aircraft/compile_aircraft_sitle.sh
    ```

- 仿真运行：（需要连接遥控器）

  - 启动仿真环境：

    - ```
      roslaunch prometheus_future_aircraft future_aircraft.launch
      ```

  - 启动控制demo：（后续融合到一个launch文件中）

    - ```
      rosrun prometheus_future_aircraft future_aircraft
      ```





##  二、任务控制demo说明

- 控制需要有一定基础，可以熟悉了解 [控制demo模块](https://wiki.amovlab.com/public/prometheus-wiki/%E6%97%A0%E4%BA%BA%E6%9C%BA%E6%8E%A7%E5%88%B6%E6%A8%A1%E5%9D%97-uav_control/%E6%97%A0%E4%BA%BA%E6%9C%BA%E6%8E%A7%E5%88%B6%E6%A8%A1%E5%9D%97-uav_control.html) 
- 针对比赛以及实际情况，选择合适的控制接口进行控制。本demo中会使用惯性系与机体系下的位置和速度控制

### 任务状态机

- 状态机：

  - TAKEOFF    起飞状态机

    - 直接调用起飞函数

    - ```
      uav_command.header.stamp = ros::Time::now();
      uav_command.header.frame_id = "ENU";
      uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
      ```
  - SEARCH     搜寻状态机

    - 使用惯性系或者机体系下的位置控制

    - ```
      uav_command.header.stamp = ros::Time::now();
      uav_command.header.frame_id = "BODY";
      uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
      uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
      uav_command.position_ref[0] = waypoint1[0];
      uav_command.position_ref[1] = waypoint1[1];
      uav_command.position_ref[2] = waypoint1[2];
      uav_command.yaw_ref = 0.0;
      ```

    - ```
      uav_command.header.stamp = ros::Time::now();
      uav_command.header.frame_id = "ENU";
      uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
      uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
      uav_command.position_ref[0] = waypoint4[0];
      uav_command.position_ref[1] = waypoint4[1];
      uav_command.position_ref[2] = waypoint4[2];
      uav_command.yaw_ref = 0.0;
      ```

  - TRACKING    跟踪状态机

    - 使用机体系下的水平方向速度控制，垂直方向高度控制

    - ```
      //坐标系
      uav_command.header.frame_id = "BODY";
      // Move模式
      uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
      // 机体系下的速度控制
      uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY;
      uav_command.velocity_ref[0] = 0.5 * now_arucos_info.position[0];
      uav_command.velocity_ref[1] = 0.5 * now_arucos_info.position[1];
      uav_command.position_ref[2] = 1.0;
      uav_command.yaw_ref = 0.0;
      ```

  - LOST            目标丢失状态机

  - RETURN       返航状态机（返航以及降落状态机）

### 视觉端处理

- 坐标系变换说明（Prometheus\Modules\tutorial_demo\advanced\autonomous_landing\include\mission_utils.h）

  - 接受图像话题 

    ```
    /prometheus/ellipse_det
    ```

  - 识别算法发布的目标位置位于**相机坐标系**（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）

  - 首先，从相机坐标系转换至**机体坐标系**（从机体往前看，物体在相机前方x为正，左方y为正，上方z为正）：`camera_offset`为相机安装偏移量，此处为下置摄像头，参看  `p450_future_aircraft.sdf`可知，相机安装于机体质心下方0.05米，因此，`camera_offset[0] = 0.0`，`camera_offset[1] = 0.0`，`camera_offset[2] = -0.05` 。

    - ```
      ellipse_det.pos_body_frame[0] = -ellipse_det.Detection_info.position[1] + camera_offset[0];
      ellipse_det.pos_body_frame[1] = -ellipse_det.Detection_info.position[0] + camera_offset[1];
      ellipse_det.pos_body_frame[2] = -ellipse_det.Detection_info.position[2] + camera_offset[2];
      ```

  - 从机体坐标系转换至**与机体固连的ENU系**（原点位于质心，x轴指向yaw=0的方向，y轴指向yaw=90的方向，z轴指向上的坐标系）：直接乘上机体系到惯性系的旋转矩阵即可 R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);

    - ```
      ellipse_det.pos_body_enu_frame = R_Body_to_ENU * ellipse_det.pos_body_frame;
      ```

  - 机体惯性系 再变化为 惯性系

    - ```
      ellipse_det.pos_enu_frame[0] = uav_state.position[0] + ellipse_det.pos_body_enu_frame[0];
      ellipse_det.pos_enu_frame[1] = uav_state.position[1] + ellipse_det.pos_body_enu_frame[1];
      ellipse_det.pos_enu_frame[2] = uav_state.position[2] + ellipse_det.pos_body_enu_frame[2];
      ```

      

## 三、无人机视觉

- 需要有一定基础
- 完成比赛中不一定会用到所有讲解的知识
- 才疏学浅，如果有大佬发现错误，欢迎留言指正

### Prometheus视觉模块简介

1. [概览](https://wiki.amovlab.com/public/prometheus-wiki/%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E6%A8%A1%E5%9D%97-object_detection/%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E6%A8%A1%E5%9D%97%E4%BB%8B%E7%BB%8D/%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E6%A8%A1%E5%9D%97%E4%BB%8B%E7%BB%8D.html)

2. 椭圆识别演示

电脑插上摄像头，输入一下命令运行，默认读取ID为0的摄像
```bash
roslaunch prometheus_detection ellipse_det.launch
```
![Peek 2022-07-22 10-33.gif](https://qiniu.md.amovlab.com/img/m/202207/20220722/1156295734737485863223296.gif)



### 椭圆检测流程简介

OPENCV版本: 霍夫椭圆检测,更慢
<img src=https://qiniu.md.amovlab.com/img/m/202207/20220722/1146163162603317041725440.png width=1000 />

1. 图像去噪声, 去除图像中到椒盐噪声
2. 弧检测,挑选出可能为弧的对象
3. 弧分类,判定弧属于四个象限中的那个一个

<img src=https://qiniu.md.amovlab.com/img/m/202207/20220722/1148073625111206118719488.png width=400 />

4. 弧过滤,运用两段弧约束, CNC约束(三段弧约束),过滤不满足要求的弧

5. 椭圆估计,在剩下的四个象限的弧中进行排列组合,使用优化算法,通过4个弧线估计一个椭圆

6. 椭圆打分,使用特定打分算法,计算椭圆与4个弧线的拟合程度,给椭圆打分,最后选出得分较高的椭圆

### 相机模型简介

世界平面到图像平面(根据小孔成像原理)

<img src=https://qiniu.md.amovlab.com/img/m/202207/20220721/2248597980650689921646592.png width=1000 />

图像平面到像素平面, 将图像平面原点映射到像素平面原点
- $x,y$图像尺寸, $u,v$像素尺寸
- $u_0,v_0$是图像坐标系原点到像素坐标系原点的偏移
- $dx, dy$ 每个像素在图像平面$x$和$y$方向上的物理尺寸

<img src=https://qiniu.md.amovlab.com/img/m/202207/20220721/1833531607766405921275904.png width=1000 />

世界坐标系到像素坐标系变换(_下图中,图像坐标系到像素坐标系转换矩阵添加畸变矫正参数矫正_)

<img src=https://qiniu.md.amovlab.com/img/m/202207/20220721/1756487722766173141565440.png width=1000 />

这里我们假设已知目标在相机坐标系下的位置$(X_c, Y_c, Z_c)$，将前两个变换矩阵相乘得到`camera_matrix`(也叫相机内参)，最后得到相机坐标系到像素坐标系的关系:

$$
\begin{bmatrix}
 u\\
 v\\
 1
\end{bmatrix}=
\begin{bmatrix}
\frac{f}{dx} & 0 & u_{0} \\
 0 & \frac{f}{dy} & v_{0}\\
 0 & 0 & 1
\end{bmatrix}\begin{bmatrix}
 \frac{X_{c}}{Z_{c}} \\
 \frac{Y_{c}}{Z_{c}} \\
 1 
\end{bmatrix}
$$

实际中，一般像素坐标$u,v$以及$Z_c$已知，通过目标的像素位置、相机内参和深度信息，就可以反解目标世界坐标，通过上述公式，变换可得:
$$
\begin{array}{c}
X_c = Z_c * (u - u_0) /(\frac{f}{dx}) \\
Y_c = Z_c * (v - v_0) /(\frac{f}{dy}) \\
\end{array}
$$

其中$Z_c$一般可以通过传感器直接获得，或者通过事先已知目标真实尺寸，像素尺寸通过相似三角形比值关系获得，具体可以看[配置目标的实际长宽](https://wiki.amovlab.com/public/prometheus-wiki/%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E6%A8%A1%E5%9D%97-object_detection/%E6%89%A9%E5%B1%95%E9%98%85%E8%AF%BB/%E9%85%8D%E7%BD%AE%E7%9B%AE%E6%A0%87%E7%9A%84%E5%AE%9E%E9%99%85%E9%95%BF%E5%AE%BD.html)进行学习。

### 相机标定
> [相机标定程序](https://wiki.amovlab.com/public/prometheus-wiki/%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E6%A8%A1%E5%9D%97-object_detection/%E6%89%A9%E5%B1%95%E9%98%85%E8%AF%BB/%E7%9B%B8%E6%9C%BA%E6%A0%87%E5%AE%9A.html)
1. 获取相机内参
2. 去除相机畸变

<img src=https://qiniu.md.amovlab.com/img/m/202207/20220721/2256346071955137046347776.png width=1000 />



某个相机的标定文件
```yaml
%YAML:1.0
---
calibration_time: "Thu 14 Jul 2022 11:46:13 AM CST"
image_width: 640
image_height: 480
flags: 0
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 6.7570491275428003e+02, 0., 3.4702214961185257e+02, 0.,
       6.7652907115648509e+02, 2.5917548194106814e+02, 0., 0., 1. ]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ -4.3154114726977300e-01, 2.7199122166782413e-01,
       -1.5471282947090615e-03, -6.6196646287719843e-04,
       -4.9889322933567892e-01 ]
avg_reprojection_error: 4.7592643246496424e-01
```

### 代码解析

[ellipse_det.cpp](../object_detection/cpp_nodes/ellipse_det.cpp)

- 整个代码逻辑
- 可调节参数
- 去除图像畸变
- 目标位置估计
- 区分起飞点和靶标

### messge定义

[DetectionInfo](../common/prometheus_msgs/msg/DetectionInfo.msg)

[MultiDetectionInfo](../common/prometheus_msgs/msg/MultiDetectionInfo.msg)

### 椭圆检测launch文件
```
<node pkg="prometheus_detection" type="ellipse_det" name="ellipse_det" output="screen">
    <param name="input_image_topic" type="string" value="/prometheus/sensor/monocular_down/image_raw" />
    <param name="camera_height_topic" type="string" value="/uav/hgt" />
    <param name="camera_params" type="string" value="$(find prometheus_future_aircraft)/conf/calib_sitl.yaml"/>
</node>
```

### 问题

- 飞行过程中无人机姿态变化，会导致估计的目标位置误差加大，该怎么解决？
