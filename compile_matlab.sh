#!/bin/bash
# 脚本名称: compile_matlab.sh
# 脚本描述: 编译Prometheus matlab_bridge 模块

# 编译基础模块
catkin_make --source Modules/common --build build/common
# 编译matlab_bridge模块
catkin_make --source Modules/matlab_bridge --build build/matlab_bridge

