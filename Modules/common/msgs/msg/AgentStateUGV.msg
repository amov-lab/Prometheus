std_msgs/Header header

## agent 在同一类别中的编号 
uint8 agent_id
## agent 在整个班组里的编号 士兵1-10 无人机 11-20 无人车 21-30
uint8 agent_id_in_team

## agent状态
uint8 state
# enum agent状态枚举
uint8 unknown=0
uint8 ready=1
uint8 dead=2
uint8 lost=3

## 机载电脑是否连接上飞控，true已连接，false则不是
bool connected
## odom失效
bool odom_valid

## 状态量：位置、速度、姿态
float32[3] position                 ## [m]
float32[3] velocity                 ## [m/s]
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数
float32[3] attitude_rate            ## [rad/s]
float32 battery_state               ## 电池状态    #float32