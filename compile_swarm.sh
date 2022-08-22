#!/bin/bash
# 脚本名称: compile_swarm.sh
# 脚本描述: 编译Prometheus swarm_control 模块

# 编译swarm_control模块
catkin_make --source Modules/swarm_control --build build/swarm_control

