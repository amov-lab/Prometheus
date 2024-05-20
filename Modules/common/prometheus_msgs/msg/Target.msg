
## 目标中心的水平位置 [0,1]
float32 cx
## 目标中心的垂直位置 [0,1]
float32 cy

## 目标宽度/图像宽度 [0,1]
float32 w
## 目标高度/图像高度 [0,1]
float32 h

## 目标置信度 [0,1]
float32 score

## 目标类别名称
string category
## 目标类别编号 从0开始计算
int32 category_id

## 目标跟踪ID号
int32 tracked_id

## 目标3D位置 单位：m
float32 px
float32 py
float32 pz

## 目标中心点的视线角度 [相机系下：右方x角度为正，下方y角度为正] 单位：度
float32 los_ax
float32 los_ay

## 偏航角误差 [图像系下：垂直向上为0°，右半圈顺时针0~180°，左半圈逆时针0~-180°]
float32 yaw_a

## detection(false/0) or tracked(true/1)
bool mode
