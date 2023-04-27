#!/bin/bash
# 脚本名称: compile_ARM.sh
# 脚本描述: 编译基础,控制，matlab模块，针对ARM平台有些编译性能不足，编译卡住，所以采用单核编译

# 编译基础模块
catkin_make --source Modules/common --build build/common -j1

# 编译控制模块
catkin_make --source Modules/uav_control --build build/uav_control -j1

# 编译通信模块
catkin_make --source Modules/communication --build build/communication -j1

#编译matlab模块
catkin_make --source Modules/matlab_bridge --build build/prometheus_matlab -j1