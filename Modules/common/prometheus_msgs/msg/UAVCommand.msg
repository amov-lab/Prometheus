std_msgs/Header header

## 控制命令的模式 
uint8 Agent_CMD
# Agent_CMD 枚举
uint8 Init_Pos_Hover=1
uint8 Current_Pos_Hover=2
uint8 Land=3
uint8 Move=4
uint8 User_Mode1=5

## 移动命令下的子模式
uint8 Move_mode
## 移动命令下的子模式枚举
uint8 XYZ_POS = 0               ### 惯性系定点控制
uint8 XY_VEL_Z_POS = 1          ### 惯性系定高速度控制
uint8 XYZ_VEL = 2               ### 惯性系速度控制
uint8 XYZ_POS_BODY = 3          ### 机体系位置控制
uint8 XYZ_VEL_BODY = 4          ### 机体系速度控制
uint8 XY_VEL_Z_POS_BODY = 5     ### 机体系定高速度控制
uint8 TRAJECTORY = 6            ### 轨迹追踪控制
uint8 XYZ_ATT = 7               ### 姿态控制（来自外部控制器）

## 控制参考量
float32[3] position_ref                ## [m]
float32[3] velocity_ref                 ## [m/s]
float32[3] acceleration_ref       ## [m/s^2]
float32 yaw_ref                             ## [rad]
float32 yaw_rate_ref                  ## [rad/s]
float32[4] att_ref       ## [rad] + [0-1]


## 控制命令的编号 防止接收到错误命令， 编号应该逐次递加
uint32 Command_ID