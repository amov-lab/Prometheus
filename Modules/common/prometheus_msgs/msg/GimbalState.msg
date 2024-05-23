Header header

# gimbal type; value = {SV::GimbalType}
uint8 type
# gimbal movement mode; 0:None; 1:lock head; 2：unlock head; 3: tracking;
uint8 moveMode
# gimbal image type; 0:normal; 1:infrared; 2:merge;
uint8 imageMode
# gimbal zoom
bool isZoom
# gimbal rec; 0:none; 1:gimbal rec; 2:local rec; 3:gimbal & local rec;
uint8 isRec
# gimbal fov, XY
float32[2] fov
# gimbal real time angle, RPY <deg>
float32[3] angleRT
# gimbal real time angle rate, RPY <deg/s> <if gimbal supported>
float32[3] angleRTRate
# gimbal setted max angle rate <deg/s> <if gimbal supported>
float32[3] angleRateSet