#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
import cv2
import os
import yaml
import math
from geometry_msgs.msg import Pose, Point, Quaternion
import time


rospy.init_node('angle_est_stat', anonymous=True)
global angle_list, time_s
angle_list = []
time_s = time.time()


def angle_callback(detmsg):
    global angle_list, time_s
    if detmsg.position.y > 0:
        angle = detmsg.position.x
        angle_list.append(angle)
        d_s = time.time() - time_s
        if d_s > 3.:
            time_s = time.time()
            n_dat = len(angle_list)
            dat = np.array(angle_list)
            dat_mean = np.mean(dat)
            dat_std = np.std(dat)
            print("mean: {:.3f}, std: {:.3f}, n_points: {}".format(dat_mean, dat_std, n_dat))
            angle_list = []


if __name__ == '__main__':
    rospy.Subscriber('/prometheus/object_detection/color_line_angle', Pose, angle_callback)
    rospy.spin()
