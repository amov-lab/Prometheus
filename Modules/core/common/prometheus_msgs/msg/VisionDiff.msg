uint8 Id
uint8 detect

uint16 objectX # 左上角
uint16 objectY # 左上角
uint16 objectWidth
uint16 objectHeight

uint16 frameWidth
uint16 frameHeight

# Gimbal 跟踪pid
float32 kp
float32 ki
float32 kd

int8 ctlMode # 0: yaw+pitch, 1: roll+pitch 3:混合(未实现)
int8 yawPitch = 0
int8 rollPitch = 1
int8 mix=3

# 用于自动缩放
float32 currSize    #框选近大远小
float32 maxSize
float32 minSize #框选大小

float32 trackIgnoreError
bool autoZoom