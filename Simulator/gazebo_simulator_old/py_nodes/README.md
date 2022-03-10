**keyboard_control_px4.py**脚本目的是在仿真过程中用键盘模拟遥控器

注：脚本不能单独使用,需要在QGC中配置遥控器通道相关参数，具体参数如下

```
RC2_TRIM = 1000us
COM_FLTMODE1 = Position
RC_CHAN_CNT = 8
RC_MAP_FLTMODE = Channel 5
RC_MAP_PITCH = Channel 3
RC_MAP_ROLL= Channel 1
RC_MAP_THROTTLE = Channel 2
RC_MAP_YAW = Channel 4
```

