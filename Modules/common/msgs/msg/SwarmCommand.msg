std_msgs/Header header

## 消息来源
string source

## 命令
uint8 Swarm_CMD
# enum CMD 控制模式枚举
uint8 Init=0
uint8 Start=1
uint8 Hold=2
uint8 Stop=3
uint8 Formation=4

uint8 Follow=11
uint8 Search=12
uint8 Attack=13

## 编队控制参考量 
float32[2] leader_pos
float32[2] leader_vel
float32 swarm_size
uint8 swarm_shape
uint8 One_column=0
uint8 Triangle=1
uint8 Square=2
uint8 Circular=3

## 搜索控制参考量 
float32 target_area_x_min          ## [m]
float32 target_area_y_min          ## [m]
float32 target_area_x_max          ## [m]
float32 target_area_y_max          ## [m]

## 攻击控制参考量 
float32[3] attack_target_pos          ## [m]
