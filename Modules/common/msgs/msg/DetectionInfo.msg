std_msgs/Header header

string object_name

## 是否检测到目标
bool detected

## 0表示相机坐标系, 1表示机体坐标系, 2表示惯性坐标系
int32 frame

## 目标位置[相机系下：右方x为正，下方y为正，前方z为正]
float32[3] position

## 目标姿态-欧拉角-(z,y,x)
float32[3] attitude

## 目标姿态-四元数-(qx,qy,qz,qw)
float32[4] attitude_q

## 视线角度[相机系下：右方x角度为正，下方y角度为正]
float32[2] sight_angle

## 偏航角误差
float32 yaw_error

## 类别
int32 category

