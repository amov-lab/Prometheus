std_msgs/Header header

#模式：0：空闲  2.simaRPN  3.deepsort/sort
uint8 mode

## 检测到的目标数量
int32 num_objs

## 每个目标的检测结果
DetectionInfo[] detection_infos
