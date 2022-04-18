#!/usr/bin/env python
# coding: utf-8
# 
import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn as rc

pubRC = None
msg = None
neutral_speed = 1500

# joystick -0.924-0.935
def convert_joy_units(data):
    #This takes the float value from -1.0 to 1.0 and converts it to a value between 1000 and 2000
    return int((data * 500) + neutral_speed)
    
# 1 roll channel,right joystick,small 2 big,1000-2000
# 2 pitch channel,right joystick,big 2 small,1000-2000
# 3 throlle channel,left joystick,big 2 small,1000-2000
# 4 yaw channel,left joystick,small 2 big,1000-2000
# 5 flight mode channel,SWC joystick,1000,1500,2000
# 6 offboard mode channel,SWD joystick,1000,2000
# 7 emergency Stop channel,SWA joystick,1000,2000
# 8 TBD channel,SWB joystick,1000,1500,2000
def joy_callback(data):
    global pubRC, msg
    (roll, pitch, throttle, yaw, _, _) = data.axes
    (b0, b1, b2, b3, b4, b5, _, _, _, _) = data.buttons
    msg = rc()
    msg.channels[0] = convert_joy_units(roll * -1)
    msg.channels[1] = convert_joy_units(pitch * -1)
    msg.channels[2] = convert_joy_units(throttle * -1)
    msg.channels[3] = convert_joy_units(yaw * -1)
    if b0 > 0:
        msg.channels[6] = neutral_speed - 500
    else:
        msg.channels[6] = neutral_speed + 500
    if b5 > 0:
        msg.channels[5] = neutral_speed + 500
    else:
        msg.channels[5] = neutral_speed - 500
    if b2 > 0:
        msg.channels[7] = neutral_speed - 500
    else:
        if b1 > 0:
            msg.channels[7] = neutral_speed + 500
        else:
            msg.channels[7] = neutral_speed
    if b4 > 0:
        msg.channels[4] = neutral_speed - 500
    else:
        if b3 > 0:
            msg.channels[4] = neutral_speed + 500
        else:
            msg.channels[4] = neutral_speed
    pubRC.publish(msg)

def init_joy_control():
    global pubRC
    rospy.init_node('joy_translator', anonymous=True)
    rospy.Subscriber("/joy", Joy, joy_callback)
    pubRC = rospy.Publisher('mavros/rc/override', rc, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
  
if __name__ == '__main__':
    try:
        init_joy_control()
    except rospy.ROSInitException:
        pass