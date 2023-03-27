# GFKD

## TODO
- **更改地图**
  - 符合真实环境
  - 无人机起飞点为原点（匹配真实情况）
  - 增加待识别的物体
  - 重新给定目标点，进行仿真测试
  - 了解octomap和ego相关参数，代码是重点
  - NUC-GPU+ubuntu2004存在gazebo仿真时间的问题(已解决)
  - NUC+ubuntu1804存在gazeboFPS的问题
  - 突然出现了飞机莫名其妙自己降落，显示为找不到遥控器，且无offboard模式(ev ght time out)
    - 这个问题一直没有解决，改用P450+outdoor就不存在问题了，有些奇怪 

## Prometheus代码共性问题
- 为什么 mavros/state才1Hz? -> 导致prometheus/uav_state更新频率很慢
- maximum_vel_error_for_vision 这个是最大位置误差，不是速度误差


- 为什么uav_estimator中获取ros时间是混乱的，一会是仿真时间，一会是系统时间


## octomap

sudo apt-get install ros-melodic-octomap-rviz-plugins
sudo apt-get install ros-melodic-octomap-*


octomap_server:https://mp.weixin.qq.com/s?__biz=MzUzNjA3ODI1Mw==&mid=2247486171&idx=1&sn=27312d4b333a266d6a991d0c61817f50&chksm=fafaf819cd8d710fd8d67fb8fea47e4aa51fac6ca6ece627ee895f0892655f14371521144443&scene=27

## 关于地图构建的说明

使用激光雷达进行地图构建
- laser_to_pointcloud节点
  - 激光雷达数据转换为点云数据（机体系）
- octomap_server节点
  - 输入：点云数据（机体系）+TF数据
  - 输出：增量式点云数据（惯性系）
- grid_map节点
  - 输入：增量式点云数据（惯性系）
  - 会对输入点云进行截断处理，并逐点膨胀
  - todo
  - 输出：膨胀后的点云（发布的时候只会发布一部分）


gridmap参考资料：https://blog.csdn.net/weixin_42284263/article/details/122283727
## 运行

cd Prometheus/GFKD_scripts
./simulation.sh

飞机加载完毕后，检查报错，然后解锁-切换至COMMAND_CONTROL模式(多机模式时，单个遥控器控制所有)，无人机自动起飞

飞机稳定后，发布触发话题执行路径规划（此时为执行预设目标点，不能在rviz中设置目标点）
rostopic pub /uav1/ego_trigger

## 如何发布目标点

 - 使用RVIZ的插件
 - 使用rosrun ego_planner pub_goal 发布任意目标点
    - x和y都等于99.99时为特殊指令，无人机原地悬停，等待下一个目标点
 - 使用roslaunch ego_planner pub_preset_goal.launch 发布预设的目标点（默认的目标点可以在launch文件中修改，目前默认是四架飞机）


## 注意事项
 - 目前scan和点云是同时订阅的，因此只能发布一个使用，不能同时发布，会有bug
 - 真实情况的时候，scan数据可能要做一个滤波，去掉检测到自身的点，不然会认为自己的位置有障碍
