#!/usr/bin/env python
# -*- coding: utf-8 -*-
#author : bingo
#email  : bingobin.lw@gmail.com
#description  :  px4 controlled by keyboard 
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import rospy
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
import time
import math

msg = """
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
If you use this script, please read readme.md first.
dir:some/src/simulation/scripts/README.md
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%%%%%%%%%%%%%%%%%%%%%%%
action_control:
%%%%%%%%%%%%%%%%%%%%%%%
        e    
   s    d    f
e and d : pitch control
s and f : roll control
---------------------------
        i    
   j    k    l
i and k : throttle control
j and l : yaw control
---------------------------
g : Speed reduction
h : Speed increase
---------------------------
%%%%%%%%%%%%%%%%%%%%%%%
command_cotrol
%%%%%%%%%%%%%%%%%%%%%%%
0 : ARM
1 : TAKEOFF
2 : OFFBOARDS
3 : LAND
4 : POSCTL
5 : ATTITCTL
---------------------------

CTRL-C to quit

"""
speed_control = 1600;
cur_target_rc_yaw = OverrideRCIn()
mavros_state = State()
armServer = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
setModeServer = rospy.ServiceProxy('/mavros/set_mode', SetMode)
local_target_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=100)
def __init__():
	rospy.init_node('PX4_control' ,anonymous=True)
	rospy.Subscriber("/mavros/state", State, mavros_state_callback)
	print("Initialized")
def RCInOverride(channel0,channel1,channel2,channel3):
	target_RC_yaw = OverrideRCIn()
	target_RC_yaw.channels[0] = channel0
	target_RC_yaw.channels[1] = channel1
	target_RC_yaw.channels[2] = channel2
	target_RC_yaw.channels[3] = channel3
	target_RC_yaw.channels[4] = 1100
	return target_RC_yaw

def mavros_state_callback(msg):
	global mavros_state
	mavros_state = msg
#	if mavros_state.armed == 0 :
#		print("disarm")

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def command_control():
	if key == '0':
		if armServer(True) :
			print("Vehicle arming succeed!")
		else:
			print("Vehicle arming failed!")
	elif key == '1':
		if setModeServer(custom_mode='AUTO.TAKEOFF'):
			print("Vehicle takeoff succeed!")
		else:
			print("Vehicle takeoff failed!")
	elif key == '2':
		if setModeServer(custom_mode='OFFBOARD'):
			print("Vehicle offboard succeed!")
		else:
			print("Vehicle offboard failed!")
	elif key == '3':
		if setModeServer(custom_mode='AUTO.LAND'):
			print("Vehicle land succeed!")
		else:
			print("Vehicle land failed!")
	elif key == '4':
		if setModeServer(custom_mode='POSCTL'):
			print("Vehicle posControl succeed!")
		else:
			print("Vehicle posControl failed!")
	elif key == '5':
		if setModeServer(custom_mode='STABILIZED'):
			print("Vehicle stabilized succeed!")
		else:
			print("Vehicle stabilized failed!")

def action_control():
	global speed_control
	#throttle
	if mavros_state.mode == 'POSCTL':
		if key == 'i'or key == 'I':
			channel1 = 1700
		elif key == 'k'or key == 'K':
			channel1 = 1300
		else :
			channel1 = 1500
	elif mavros_state.mode == 'STABILIZED':
		if key == 'i'or key == 'I':
			channel1 = 1400
		elif key == 'k'or key == 'K':
			channel1 = 1200
		else :
			channel1 = 1200
	else:
		channel1 = 1000
	#pitch
	if key == 'e' or key == 'E':
		channel2 = speed_control
	elif key == 'd' or key == 'D':
		channel2 = 3000-speed_control
	else:
		channel2 = 1500
	#roll
	if key == 's' or key == 'S':
		channel0 = 3000-speed_control
	elif key == 'f' or key == 'F':
		channel0 = speed_control
	else:
		channel0 = 1500
	#yaw
	if key == 'j' or key == 'J':
		channel3 = 1400
	elif key == 'l' or key == 'L':
		channel3 = 1600
	else:
		channel3 = 1500
	global cur_target_rc_yaw
	cur_target_rc_yaw = RCInOverride(channel0,channel1,channel2,channel3)
	if key == 'h' or key == 'H':
		speed_control = speed_control + 10
		print('Current control speed :',speed_control)
	elif key == 'g' or key == 'G':
		speed_control = speed_control - 10
		print('Current control speed :',speed_control)
if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	print (msg)
	__init__()
	cur_target_rc_yaw = RCInOverride(1500,1500,1000,1500)
	while(1):
		key= getKey()
		command_control()
		action_control()
		local_target_pub.publish(cur_target_rc_yaw)
		if (key == '\x03'):
			break
#		rospy.spin()
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
