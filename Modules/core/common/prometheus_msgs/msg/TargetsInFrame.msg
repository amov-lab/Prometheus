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

##focusing solely on tracking , without considering detection
int32 no_track_frame_count

## 是否处于跟踪状态，0：否，1：是。当状态为1，targets为空表示丢失；targets不为空表示正常跟踪。
bool tracking_state
