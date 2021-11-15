#!/usr/bin/env python
#coding=utf-8
from __future__ import print_function
import rospy
from prometheus_msgs.msg import GimbalCtl
import sys, select, termios, tty


msg_tip = """
Reading from the keyboard  and Publishing to Cam_Ctr!
---------------------------
Camera Control:

float32 pitch 'q a'
float32 yaw   'w s'
float32 zoom  'e d'
float32 focus  'r f' 
uint16  home  'z'
uint16  TakePicture 'x'
uint16  cameraModeChange 'c'
uint16  yawfollow 'y'

uint16  home 'h' "three hit"
---------------------------
CTRL-C to quit
"""


StatusBindings = {
        'z':(1,0,0,0),	#home
        'x':(0,1,0,0),	#takePicte
    	'c':(0,0,1,0),	#ModeChange
		'y':(0,0,0,1),	#yaw follow
		'h':(1,0,0,0)	#quad home
    }

ValueBindings={
        'q':(1,0,0,0),	#pithc
        'w':(0,1,0,0),	#yaw
        'e':(0,0,1,0),  #zoom
        'r':(0,0,0,1), 	#focus
		'a':(-1,0,0,0),
        's':(0,-1,0,0),
        'd':(0,0,-1,0),
        'f':(0,0,0,-1),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
 
 
if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	pub = rospy.Publisher('/GimbalCtl', GimbalCtl, queue_size = 1)
	rospy.init_node('Send_Cam_Ctr_Message', anonymous = True)
	pitch_speed = rospy.get_param("~pitch_speed", 10)
	yaw_speed = rospy.get_param("~yaw_speed", 10)
	zoom_speed = rospy.get_param("~zoom_speed", 1)
	focus_speed = rospy.get_param("~focus_speed", 1)
	print(msg_tip)
	#print(pitch_speed)
	status = 0
	try:
		while(1):
			msg = GimbalCtl()
			#print(msg_tip)
			key = getKey()
			#rospy.loginfo(key)
			if key in StatusBindings.keys():
				msg.home = StatusBindings[key][0]
				msg.TakePicture = StatusBindings[key][1]
				msg.cameraModeChange = StatusBindings[key][2]
				msg.yawfollow = StatusBindings[key][3]
				msg.quadhome = StatusBindings[key][0]

			elif key in ValueBindings.keys():
				msg.pitch = ValueBindings[key][0]*pitch_speed
				msg.yaw = ValueBindings[key][1]*yaw_speed
				msg.zoom = ValueBindings[key][2]*zoom_speed
				msg.focus = ValueBindings[key][3]*focus_speed


			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break
			#print(msg)
			pub.publish(msg)

	except Exception as e:
		print(e)

	finally:
		pub.publish(msg)
		rospy.loginfo(msg)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



