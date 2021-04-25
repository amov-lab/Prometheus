#!/usr/bin/env python
import sys
import cv2
import os.path
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import signal

rospy.init_node('video_replay', anonymous=True)


def exit(signum, frame):
    print('You choose to stop me.')
    exit()


def video_replay(input_video_dir, t, pub_topic):    
    pub = rospy.Publisher(pub_topic, Image, queue_size=10)
    rate = rospy.Rate(t)
    pathDir = os.listdir(input_video_dir)

    signal.signal(signal.SIGINT, exit)
    signal.signal(signal.SIGTERM, exit)

    for allDir in pathDir:
        videopath = os.path.join(input_video_dir, allDir)        
        cap = cv2.VideoCapture(videopath)

        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                bridge = CvBridge()                
                msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                pub.publish(msg)
            else:
                rospy.loginfo("video replay failed.")
                cv2.destroyAllWindows()

            rate.sleep()


if __name__ == '__main__':
    input_video_dir = rospy.get_param('~input_video_dir', '/home/nvidia/Prometheus/video')
    t = rospy.get_param('~video_rate', 20)
    pub_topic = rospy.get_param('~publish_topic', '/prometheus/object_detection/video_replay')
    
    try:
        video_replay(input_video_dir, t, pub_topic)
    except rospy.ROSInterruptException:
        pass
