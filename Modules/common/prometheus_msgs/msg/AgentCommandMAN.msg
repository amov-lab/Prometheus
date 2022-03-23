std_msgs/Header header

## 控制命令的模式 
uint8 Agent_CMD
# Agent_CMD 枚举
uint8 Idle=0
uint8 Takeoff=1
uint8 Hold=2
uint8 Land=3
uint8 Move=4
uint8 User_Mode1=5

## 移动命令下的子模式
uint8 Move_mode
## 移动命令下的子模式枚举
uint8 XYZ_POS = 0                       ### 定点
uint8 XY_VEL_Z_POS = 1          ### 定高
uint8 XYZ_VEL = 3                        ### 速度控制
uint8 XYZ_ACC = 4                       ### 加速度控制
uint8 TRAJECTORY = 5              ### 轨迹控制
uint8 XYZ_POS_BODY = 6  
uint8 XYZ_VEL_BODY = 7 
uint8 XY_VEL_Z_POS_BODY = 8

## 控制参考量
float32[3] position_ref                ## [m]
float32[3] velocity_ref                 ## [m/s]
float32[3] acceleration_ref       ## [m/s^2]
float32 yaw_ref                             ## [rad]
float32 yaw_rate_ref                  ## [rad/s]

## 控制命令的编号 防止接收到错误命令， 编号应该逐次递加
uint32 Command_ID