std_msgs/Header header

## 控制命令的模式 
uint8 Agent_CMD
# Agent_CMD 枚举
uint8 Idle=0
uint8 Hold=1
uint8 Direct_Control_BODY=2
uint8 Direct_Control_ENU=3
uint8 Point_Control=4
uint8 Path_Control=5
uint8 Stop=6

## 控制参考量
float32[2] position_ref     ## [m]
float32[2] velocity_ref     ## [m]

float32 yaw_ref             ## [rad]
float32 angular_vel         ## [rad/s]
