#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import os

from cv_bridge import CvBridge, CvBridgeError
import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
import yaml


import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')  
from gi.repository import GObject, Gst, GstRtspServer
 


def main(width,height,port,factory_name):
    global out_send
    out_send = cv2.VideoWriter('appsrc is-live=true ! videoconvert ! \
                                omxh264enc bitrate=12000000 ! video/x-h264, \
                                stream-format=byte-stream ! rtph264pay pt=96 ! \
                                udpsink host=127.0.0.1 port=5400 async=false',
                                cv2.CAP_GSTREAMER, 0, 30, (width,height), True)
 
    if not out_send.isOpened():
        print('VideoWriter not opened')
        # exit(0)
 
    rtsp_port_num = port  
 
    server = GstRtspServer.RTSPServer.new()
    server.props.service = "%d" % rtsp_port_num
    server.attach(None)
    
    factory = GstRtspServer.RTSPMediaFactory.new()
    factory.set_launch("(udpsrc name=pay0 port=5400 buffer-size=524288 \
                        caps=\"application/x-rtp, media=video, clock-rate=90000, \
                        encoding-name=(string)H264, payload=96 \")")
                  
    factory.set_shared(True)
    server.get_mount_points().add_factory(factory_name, factory)

    print("\n *** Launched RTSP Streaming at rtsp://localhost:%d/demo \n" % rtsp_port_num)    

 
def callback(data):
    scaling_factor = 0.5
    global count,bridge
    global out_send
    count = count + 1
    if count == 1:
        count = 0
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        out_send.write(cv_img)
        # cv2.imshow("frame" , cv_img)
        cv2.waitKey(3)
    else:
        pass
 

 
if __name__ == '__main__':
    rospy.init_node('gstreamer_rtsp_stream', anonymous=True)
    input_topic = rospy.get_param('~camera_topic', '/prometheus/camera/rgb/image_raw')
    config = rospy.get_param('~config_info', 'encode_config.yaml')

    yaml_config_fn = config
    with open(yaml_config_fn) as f:
        yaml_config=yaml.load(f,Loader=yaml.FullLoader)

    image_width=yaml_config['image_width']
    image_height=yaml_config['image_height']
    rtsp_port=yaml_config['rtsp_port']
    factory_name=yaml_config['factory_name']

    global count,bridge
    global out_send
    count = 0
    bridge = CvBridge()
    main(image_width,image_height,rtsp_port,factory_name)
   
    rospy.Subscriber(input_topic, Image, callback)
    rospy.spin()




