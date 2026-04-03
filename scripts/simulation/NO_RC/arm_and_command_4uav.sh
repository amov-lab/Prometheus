#!/bin/bash

# 解锁1无人机
gnome-terminal --window -e 'rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 0
arming: true 
px4_mode: \"\"
control_state: \"\""' 

# 解锁2无人机
gnome-terminal --window -e 'rostopic pub -1 /uav2/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 0
arming: true 
px4_mode: \"\"
control_state: \"\""' 

# 解锁3无人机
gnome-terminal --window -e 'rostopic pub -1 /uav3/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 0
arming: true 
px4_mode: \"\"
control_state: \"\""' 

# 解锁4无人机
gnome-terminal --window -e 'rostopic pub -1 /uav4/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 0
arming: true 
px4_mode: \"\"
control_state: \"\""' 

# 暂停 3 秒
sleep 3

# 进入1命令模式
gnome-terminal --window -e 'rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 3
arming: true 
px4_mode: \"\"
control_state: \"COMMAND_CONTROL\""' 

# 进入2命令模式
gnome-terminal --window -e 'rostopic pub -1 /uav2/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 3
arming: true 
px4_mode: \"\"
control_state: \"COMMAND_CONTROL\""' 

# 进入3命令模式
gnome-terminal --window -e 'rostopic pub -1 /uav3/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 3
arming: true 
px4_mode: \"\"
control_state: \"COMMAND_CONTROL\""' 

# 进入4命令模式
gnome-terminal --window -e 'rostopic pub -1 /uav4/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 3
arming: true 
px4_mode: \"\"
control_state: \"COMMAND_CONTROL\""' 
