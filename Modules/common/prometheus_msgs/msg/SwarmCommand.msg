std_msgs/Header header

## 消息来源
string source

## 编队套件数量
uint8 swarm_num

uint8 swarm_location_source

# enum loc 定位来源枚举
uint8 mocap = 0
uint8 gps = 4
uint8 rtk = 5
uint8 uwb = 6

## 命令
uint8 Swarm_CMD
# enum CMD 控制模式枚举
uint8 Ready=0
uint8 Init=1
uint8 Start=2
uint8 Hold=3
uint8 Stop=4
uint8 Formation=5

uint8 Col_Around_Circle=6
uint8 Col_Follow_UAV=7
uint8 Col_Follow_UGV=8

uint8 Follow=11
uint8 Search=12
uint8 Attack=13

## 编队控制参考量 
float32[3] leader_pos
float32[2] leader_vel
float32 swarm_size
uint8 swarm_shape
uint8 One_column=0
uint8 Triangle=1
uint8 Square=2
uint8 Circular=3
uint8 around_Circle=4

## 搜索控制参考量 
float32 target_area_x_min          ## [m]
float32 target_area_y_min          ## [m]
float32 target_area_x_max          ## [m]
float32 target_area_y_max          ## [m]

## 攻击控制参考量 
float32[3] attack_target_pos          ## [m]

#队形位置
geometry_msgs/Point[] formation_poses
