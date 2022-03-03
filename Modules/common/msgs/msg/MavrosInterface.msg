uint8 ARMING = 0
uint8 SET_MODE = 1
uint8 REBOOT_PX4 = 2

uint8 type
bool arming
#http://wiki.ros.org/mavros/CustomModes ,可参考该网址设置模式名,常用模式名:OFFBOARD,AUTO.LAND,AUTO.RTL,POSCTL
string mode