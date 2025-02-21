gnome-terminal --window -e 'rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
cmd: 3
arming: true 
px4_mode: ''
control_state: 'COMMAND_CONTROL'"'
