std_msgs/Header header

uint8 Mode #控制模式

##控制模式类型枚举
uint8 Stop=0
uint8 Forward=1
uint8 Left=2
uint8 Right=3
uint8 Back=4
uint8 CMD=5
uint8 Waypoint=6

float64 linear
float64 angular

## 航点
RheaGPS[] waypoint
