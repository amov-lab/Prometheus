std_msgs/Header header

## 用于控制模块的log消息，可自行增加需要记录的话题
float32 time             ## [s]

## 0代表px4_sender,1代表px4_pos_controller(姿态控制)
int32 control_type

DroneState Drone_State

ControlCommand Control_Command

ControlOutput Control_Output

AttitudeReference Attitude_Reference

geometry_msgs/PoseStamped ref_pose



