# prometheus_mission

任务模块

- 航点追踪（正方形飞行） [square.cpp]
- 自主降落 [autonomous_landing.cpp]
- 简易避障 [collision_avoidance.cpp]
- 双目简易避障 [collision_avoidance_streo.cpp]
- 编队飞行（目前仅支持gazebo仿真）[formation_control_sitl.cpp]
- 负载投掷 [payload_drop.cpp]
- 目标追踪 [target_tracking.cpp]

## 航点追踪（正方形飞行）

‘roslaunch prometheus_mission square'

 - 若需要修改正方形参数，请修改'square.yaml'。
 - 程序启动后，会等待无人机解锁及切换至OFFBOARD模式
 - 执行航点顺序为： 起飞 - Point 1 - Point 2 - Point 3 - Point 4 - Point 5 - 降落
