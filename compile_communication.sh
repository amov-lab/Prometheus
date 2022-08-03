#!/bin/bash
# 脚本名称: compile_communication.sh
# 脚本描述: 编译Prometheus通信模块

# 编译基础模块
catkin_make --source Modules/common --build build/common
# 编译通信模块
catkin_make --source Modules/communication --build build/communication
