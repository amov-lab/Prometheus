std_msgs/Header header

## 位置控制参考量

## 默认为 XYZ位置追踪模式 （sub_mode = 0）； 速度追踪启用时，控制器不考虑位置参考量及位置状态反馈
uint8 Move_mode

uint8 XYZ_POS      = 0  ##0b00
uint8 XY_POS_Z_VEL = 1  ##0b01
uint8 XY_VEL_Z_POS = 2  ##0b10
uint8 XYZ_VEL = 3       ##0b11
uint8 XYZ_ACC = 4
uint8 TRAJECTORY   = 5  

uint8 Move_frame
# 默认情况下均为ENU_FRAME,注意轨迹追踪和XYZ_ACC一定是ENU_FRAME
uint8 ENU_FRAME  = 0
uint8 BODY_FRAME = 1
uint8 MIX_FRAME = 2 ##2020.4.6 临时增加的模式，后期需要统一整合，该模式下，xy采用机体系控制，z采用enu系控制



## 时刻： 用于轨迹追踪
float32 time_from_start          ## [s]

## 参考量：位置、速度、加速度、加加速度、加加加速度
float32[3] position_ref          ## [m]
float32[3] velocity_ref          ## [m/s]
float32[3] acceleration_ref      ## [m/s^2]
## float32[3] jerk_ref              ## [m/s^3]
## float32[3] snap_ref              ## [m/s^4]

## 角度参考量：偏航角、偏航角速度、偏航角加速度
bool Yaw_Rate_Mode                      ## True 代表控制偏航角速率
float32 yaw_ref                  ## [rad]
float32 yaw_rate_ref             ## [rad/s] 
## float32 yaw_acceleration_ref     ## [rad/s] 