Header header
uint8 Id
#control mode 0:nothong 1:angle 2:speed 3:home postion 
uint8 rpyMode
uint8 manual = 1
uint8 home = 2
uint8 hold = 3 # 不控制
uint8 fellow = 4 #fellow吊舱跟随无人机移动

uint8 roll
uint8 yaw 
uint8 pitch

uint8 noCtl = 0
uint8 velocityCtl = 1
uint8 angleCtl = 2

float32 rValue # deg 单位
float32 yValue # deg
float32 pValue # deg 


# focus
uint8 focusMode # 默认值
uint8 focusStop = 1
uint8 focusOut = 2
uint8 focusIn = 3

# zoom
uint8 zoomMode # 默认值
uint8 zoomStop = 1
uint8 zoomOut = 2
uint8 zoomIn = 3