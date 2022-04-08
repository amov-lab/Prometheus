std_msgs/Header header

## 无人机编号 
uint8 uav_id

## 无人机状态
uint8 state
## enum 状态枚举
uint8 unknown=0
uint8 ready=1
uint8 dead=2
uint8 lost=3

## 无人机定位来源
uint8 location_source

## 定位来源枚举
uint8 mocap=0
uint8 t265=1
uint8 gazebo=2
uint8 fake_odom=3
uint8 gps = 4
uint8 rtk = 5
uint8 uwb = 6

## GPS状态,变量对应状态可参考mavros_msgs/GPSRAW中的fix_type
uint8 gps_status

## PX4飞控当前飞行模式  int8
string mode
## 机载电脑是否连接上飞控，true已连接，false则不是
bool connected
## 是否解锁，true为已解锁，false则不是
bool armed
## odom失效
bool odom_valid

## 无人机状态量：位置、速度、姿态
float32[3] global_position
float32[3] position                 ## [m]
float32[3] velocity                 ## [m/s]
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数
float32[3] attitude_rate            ## [rad/s]
float32 battery_state               ## 电池状态   
float32 battery_percetage         