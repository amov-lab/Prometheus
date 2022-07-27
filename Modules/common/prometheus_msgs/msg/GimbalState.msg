Header header
uint8 Id

# 0: 发一句,回一句: 此状态下，相机倍数zoomVal有效，imuAngleVel为估算直
# 1: 发一句,一直回复: 此状态下，相机倍数zoomVal失效，imuAngleVel为真直
uint8 feedbackMode

#mode 
# 0: 手动控制
# 1: home 
# 2: tracking 
# 3: yaw follow 吊舱跟随无人机移动
# 4: hold 吊舱不跟随无人机
# 5: search 自动移动旋转
uint8 mode

# 是否视频录制
bool isRecording

# 是否开启自动缩放(VisionDiff需要指定面积才能生效)
# 0: 保持
# 1: 放大
# 2: 缩小
# 3: 自动
uint8 zoomState

# 当前所处倍数
float32 zoomVal

#roll,pitch,yaw
float32[3] imuAngle

#Current gimbal joint angles(roll,pitch,yaw), published at 30 Hz.
float32[3] rotorAngle

# rpy_vel 角速度
float32[3] imuAngleVel

# rpy_tgt 目标角度
float32[3] rotorAngleTarget