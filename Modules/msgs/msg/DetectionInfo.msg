std_msgs/Header header

## 是否检测到目标
bool detected

## 0 for body frame, 1 for inertial frame
int32 frame

## 目标位置[机体系下：前方x为正，右方y为正，下方z为正]
float32[3] position

## 目标姿态
float32[3] attitude