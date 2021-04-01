std_msgs/Header header

## aruco编码
int32 aruco_num

## 是否检测到目标
bool detected

## 目标位置-全局坐标系-(x,y,z)
float32[3] position

## 目标姿态-四元数-(qx,qy,qz,qw)
float32[4] orientation

## 视线角度-相机系下-(右方x角度为正,下方y角度为正)
float32[2] sight_angle
