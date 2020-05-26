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
from prometheus_msgs.msg import DetectionInfo, MultiDetectionInfo
import time


rospy.init_node('depth_single_est_stat', anonymous=True)
global depths_list, time_s
depths_list = []
time_s = time.time()


def depth_callback(detmsg):
    global depths_list, time_s
    if detmsg.detected > 0:
        depth = detmsg.position[2]
        depths_list.append(depth)
        d_s = time.time() - time_s
        if d_s > 3.:
            time_s = time.time()
            n_dat = len(depths_list)
            dat = np.array(depths_list)
            dat_mean = np.mean(dat)
            dat_std = np.std(dat)
            print("mean: {:.3f}, std: {:.3f}, n_points: {}".format(dat_mean, dat_std, n_dat))
            depths_list = []


if __name__ == '__main__':
    rospy.Subscriber('/prometheus/object_detection/landpad_det', DetectionInfo, depth_callback)
    rospy.spin()
