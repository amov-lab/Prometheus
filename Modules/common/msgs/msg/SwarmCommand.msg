std_msgs/Header header

## 控制命令的编号 防止接收到错误命令， 编号应该逐次递加
uint32 Command_ID

## 消息来源
string source

## 控制命令的模式 
uint8 Mode
# enum Mode 控制模式枚举
uint8 Idle=0
uint8 Takeoff=1
uint8 Hold=2
uint8 Land=3
uint8 Disarm=4
uint8 Position_Control=5
uint8 Velocity_Control=6
uint8 Accel_Control=7
uint8 Move=8
uint8 User_Mode1=9

## 控制参考量 
uint8 swarm_shape

uint8 One_column=0
uint8 Triangle=1
uint8 Square=2
uint8 Circular=3

## 默认为 XYZ位置追踪模式 （sub_mode = 0）； 速度追踪启用时，控制器不考虑位置参考量及位置状态反馈
uint8 Move_mode

uint8 XYZ_POS      = 0  ##0b00
uint8 XY_POS_Z_VEL = 1  ##0b01
uint8 XY_VEL_Z_POS = 2  ##0b10
uint8 XYZ_VEL = 3       ##0b11
uint8 XYZ_ACC = 4
uint8 TRAJECTORY   = 5 


float32 swarm_size

float32[3] position_ref          ## [m]
float32[3] velocity_ref          ## [m]
float32 yaw_ref                  ## [rad]

