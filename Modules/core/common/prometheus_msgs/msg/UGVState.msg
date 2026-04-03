std_msgs/Header header

## 无人机编号
uint8 ugv_id

## 电池电量
float32 battery

## 无人机状态量：位置、速度、姿态
float32[3] position                 ## [m]
float32[3] velocity                 ## [m/s]
float32[3] attitude                 ## [rad]

geometry_msgs/Quaternion attitude_q ## 四元数
