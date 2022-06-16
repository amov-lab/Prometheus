# 多二维码识别自主降落
![多二维码.gif](https://qiniu.md.amovlab.com/img/m/202206/20220606/1130165150895711620857856.gif)
# 使用
_多个二维码检测比较吃CPU资源，可能会导致卡顿_
```bash
roslaunch prometheus_demo find_aruco_marker_all.launch
```

输入二维码id，目前world中有id为1~20二维码

# 逻辑描述
`Modules/object_detection/cpp_nodes/aruco_det.cpp`有3种运行模式`run_state`
- 0: 二维码检测
- 1: 标定map位姿，完成标定后自动进入2模式. **相机移动后需要重新标定** 
- 2: 发布全局map先对于相机的位姿，发布检测到每个二维码位姿
- 3: 通过平均值方法，计算每个二维码的世界坐标系位姿。

模式1，2应用场景：标定世界坐标位姿，然后检测其他二维码的位姿，就可以通过标定了的世界坐标系位姿，计算出其他坐标系相对于世界坐标系的位姿，_而不在是相机坐标系的位姿_。

本程序只使用`Modules/object_detection/cpp_nodes/aruco_det.cpp`中`run_state==0`模式。发发布所有二维码在相机坐标系位置。`Modules/tutorial_demo/advanced/find_aruco_marker/src/main.cpp`接收二维码信息，转化为机体系坐标，在机体坐标系控制无人机到达二维码上方悬停。



