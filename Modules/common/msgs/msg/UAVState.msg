std_msgs/Header header

## 无人机编号 
uint8 uav_id

## 无人机状态
uint8 state
# enum agent状态枚举
uint8 unknown=0
uint8 ready=1
uint8 dead=2
uint8 lost=3

## PX4飞控当前飞行模式  int8
string mode
## 机载电脑是否连接上飞控，true已连接，false则不是
bool connected
## 是否解锁，true为已解锁，false则不是
bool armed
## odom失效
bool odom_valid

## 无人机状态量：位置、速度、姿态
float32[3] position                 ## [m]
float32[3] velocity                 ## [m/s]
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数
float32[3] attitude_rate            ## [rad/s]
float32 battery_state               ## 电池状态   
float32 battery_percetage         