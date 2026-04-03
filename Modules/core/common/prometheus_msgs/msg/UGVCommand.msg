std_msgs/Header header

## 控制命令的编号 防止接收到错误命令， 编号应该逐次递加
uint32 Command_ID

## 控制命令的模式 
uint8 Mode
# enum Mode 控制模式枚举
uint8 Hold=0
uint8 Direct_Control_BODY=1
uint8 Direct_Control_ENU=2
uint8 Point_Control=3
uint8 Path_Control=4
uint8 Test=5

float32[2] position_ref     ## [m]
float32 yaw_ref             ## [rad]

float32[2] linear_vel       ## [m/s]
float32 angular_vel         ## [rad/s]


