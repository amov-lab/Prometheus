std_msgs/Header header

## 位置控制器输出： 期望推力及期望油门
float32[3] Thrust              ## [rad]  
float32[3] Throttle

## NE控制器的中间变量
float32[3] u_l                 ## [0-1] 惯性系下的油门量
float32[3] u_d                 ## [rad]
float32[3] NE                  ## [rad]
  
