std_msgs/Header header

## 
int32 agent_num
int32 man_num
int32 uav_num
int32 ugv_num

# enum 
uint8 MAN=0
uint8 UAV=1
uint8 UGV=2

## 从1开始
AgentStateMAN[] agent_state_man
UAVState[] agent_state_uav
AgentStateUGV[] agent_state_ugv