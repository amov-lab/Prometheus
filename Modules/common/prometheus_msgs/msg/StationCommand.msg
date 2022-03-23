std_msgs/Header header

## 启动标志位
uint8 flag

## 中途点
geometry_msgs/PoseStamped first_goal

## 目标点
geometry_msgs/PoseStamped goal

## 控制命令的模式 
uint8 Command
# enum Command 控制模式枚举
uint8 Start=0
uint8 Return=1
uint8 Stop=2



