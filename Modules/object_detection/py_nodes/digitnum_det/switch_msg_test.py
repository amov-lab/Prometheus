#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import Bool


rospy.init_node('switch_msg_test', anonymous=True)
pub = rospy.Publisher('/prometheus/switch/ellipse_det', Bool, queue_size=10)
rate = rospy.Rate(10) # 10hz

while True:
    pub.publish(True)
    rate.sleep()
