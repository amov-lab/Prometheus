std_msgs/Header header

## 无人机Setup类型（可用于模拟遥控器）
uint8 cmd
uint8 ARMING = 0
uint8 SET_PX4_MODE = 1
uint8 REBOOT_PX4 = 2
uint8 SET_CONTROL_MODE = 3

bool arming

## PX4模式名查询:http://wiki.ros.org/mavros/CustomModes
## 常用模式名:OFFBOARD,AUTO.LAND,AUTO.RTL,POSCTL
string px4_mode

## INIT,MANUAL_CONTROL,HOVER_CONTROL,COMMAND_CONTROL,LAND_CONTROL
string control_state
