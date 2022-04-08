std_msgs/Header header

## Setup类型（模拟遥控器）
uint8 cmd
uint8 ARMING = 0
uint8 SET_PX4_MODE = 1
uint8 REBOOT_PX4 = 2
uint8 SET_HOME = 3
uint8 SET_CONTROL_MODE = 4

bool arming

## http://wiki.ros.org/mavros/CustomModes ,可参考该网址设置模式名,常用模式名:OFFBOARD,AUTO.LAND,AUTO.RTL,POSCTL
string px4_mode

string control_mode

HomePoint home_point