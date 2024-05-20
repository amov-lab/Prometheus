## ROS Header
std_msgs/Header header

## 帧ID号
int32 frame_id

## 帧像素高度
int32 height
## 帧像素宽度
int32 width

## 当前检测/跟踪频率
float32 fps

## 当前成像设备的水平视场角，单位：度
float32 fov_x
## 当前成像设备的垂直视场角，单位：度
float32 fov_y

## 当前帧每个目标信息
Target[] targets

## 当前检测的兴趣区域，ROIs
ROI[] rois
