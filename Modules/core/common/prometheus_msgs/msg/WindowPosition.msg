std_msgs/Header header

#模式：0：空闲  1.kcf   3.deepsort/sort(点击的id) 
uint8 mode


#波门位置X,#波门位置Y(kcf,点击画面的功能的时候使用),左上角为（0,0）
int16 origin_x
int16 origin_y
int16 width
int16 height 

#波门位置X,#波门位置Y
#int16 window_position_x = origin_x + width/2
#int16 window_position_y = origin_y + height/2
int16 window_position_x
int16 window_position_y

#算法检测结果的ID
int32 track_id
