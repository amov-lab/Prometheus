std_msgs/Header header
uint8 rhea_id

## 速度
float64 linear
float64 angular

## 航向角
float64 yaw

## GPS
float32 latitude  #纬度
float32 longitude #经度
float32 altitude  #高度

float32[3] position

## Status
float32 battery_voltage