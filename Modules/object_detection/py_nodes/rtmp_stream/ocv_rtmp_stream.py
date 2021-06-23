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
import subprocess
import os
import yaml
import math


rospy.init_node('rtmp_stream', anonymous=True)

rtmp = r'rtmp://localhost:1935/live'
size_str = str(640) + 'x' + str(480)
command = ['ffmpeg',
    '-y', '-an',
    '-f', 'rawvideo',
    '-vcodec','rawvideo',
    '-pix_fmt', 'bgr24',
    '-s', size_str,
    '-r', '25',
    '-i', '-',
    '-c:v', 'libx264',
    '-pix_fmt', 'yuv420p',
    '-preset', 'ultrafast',
    '-f', 'flv',
    rtmp]
pipe = subprocess.Popen(
    command,
    shell=False,
    stdin=subprocess.PIPE
)


def image_callback(imgmsg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
    # processing
    # end

    h, w = frame.shape[:2]
    img_resize = 480
    if h > w:
        h = int(float(h) / w * img_resize)
        w = img_resize
    else:
        w = int(float(w) / h * img_resize)
        h = img_resize
    frame = cv2.resize(frame, (w, h))
    frame = frame[:, :640, :]
    pipe.stdin.write(frame.tostring())
    cv2.imshow("cap", frame)
    # cv2.imshow("area", area)
    cv2.waitKey(10)


def rtmp_stream(topic_name):
    rospy.Subscriber(topic_name, Image, image_callback)
    rospy.spin()


if __name__ == '__main__':
    subscriber = rospy.get_param('~camera_topic', '/prometheus/camera/rgb/image_raw')

    try:
        rtmp_stream(subscriber)
    except rospy.ROSInterruptException:
        pass

    pipe.terminate()

