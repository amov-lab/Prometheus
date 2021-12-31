#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import rospy
import math
import os
import cv2
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float32MultiArray
from prometheus_msgs.msg import DetectionInfo, MultiDetectionInfo



# OBJECT_NAME = "object"
# OBJECT_HEIGHT = 0.2



rospy.init_node('oneshot_detection_client', anonymous=True)


pub_topic_name = rospy.get_param('~output_topic', '/prometheus/object_detection/oneshot_det')
config = rospy.get_param('~camera_parameters', 'camera_param.yaml')
OBJECT_NAME = rospy.get_param('~object_name', 'object')
OBJECT_HEIGHT = rospy.get_param('~object_height', 0.2)  # m

# print(pub_topic_name)
# print(object_names_txt)
# print(cls_names)
# print(config)

fs = cv2.FileStorage(config, cv2.FileStorage_READ)
image_width = int(fs.getNode('image_width').real())
image_height = int(fs.getNode('image_height').real())
camera_matrix = fs.getNode('camera_matrix').mat()
distortion_coefficients = fs.getNode('distortion_coefficients').mat()
fs.release()
print(image_width)
print(image_height)
print(camera_matrix)
print(distortion_coefficients)

print(OBJECT_NAME)
print(OBJECT_HEIGHT)

pub = rospy.Publisher(pub_topic_name, MultiDetectionInfo, queue_size=1)
rate = rospy.Rate(100)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('127.0.0.1', 9096))
last_fid = 0
m_info = MultiDetectionInfo()
m_info.num_objs = 0

# print(camera_matrix[0][0])
# print(camera_matrix[1][1])
aos_x = math.atan(image_width / 2. / camera_matrix[0][0])  # angle of sight
aos_y = math.atan(image_height / 2. / camera_matrix[1][1])
# print(aos_x)
# print(aos_y)

while not rospy.is_shutdown():
    data = s.recv(62)  # 35
    data = data.decode('utf-8')
    print(data)


    if len(data) > 0:
        nums = data.split(',')
        if len(nums) == 12:
            frame_id = int(nums[0])
            deted = int(nums[1])
            order = int(nums[2])
            cls = int(nums[3])
            xmin, ymin, w, h = float(nums[4]), float(nums[5]), float(nums[6]), float(nums[7])
            score = float(nums[8])
            pixel_cx = int(nums[9])
            pixel_cy = int(nums[10])
            detect_track = int(nums[11])  # 0:detect, 1:track
            m_info.detect_or_track = detect_track
            # print(frame_id)
            if deted >= 1:
                d_info = DetectionInfo()
                d_info.detected = True
                d_info.frame = frame_id
                d_info.object_name = OBJECT_NAME  # cls_names[cls]
                d_info.category = cls
                d_info.sight_angle = [(xmin+w/2.-0.5)*aos_x, (ymin+h/2.-0.5)*aos_y]
                d_info.pixel_position = [pixel_cx, pixel_cy]
                
                if OBJECT_HEIGHT > 0:
                    depth = (OBJECT_HEIGHT*camera_matrix[1][1]) / (h*image_height)
                    d_info.position = [math.tan(d_info.sight_angle[0])*depth, math.tan(d_info.sight_angle[1])*depth, depth]
                
                m_info.detection_infos.append(d_info)
                m_info.num_objs += 1
                for i in range(deted):
                    if i > 0:
                        data = s.recv(62)  # 35
                        data = data.decode('utf-8')
                        print(data)
                        if len(data) > 0:
                            nums = data.split(',')
                            if len(nums) == 11:
                                frame_id = int(nums[0])
                                deted = int(nums[1])
                                order = int(nums[2])
                                cls = int(nums[3])
                                xmin, ymin, w, h = float(nums[4]), float(nums[5]), float(nums[6]), float(nums[7])
                                score = float(nums[8])
                                pixel_cx = int(nums[9])
                                pixel_cy = int(nums[10])
                                detect_track = int(nums[11])  # 0:detect, 1:track
                                assert order == i, "server error"
                                d_info = DetectionInfo()
                                d_info.detected = True
                                d_info.frame = frame_id
                                d_info.object_name = OBJECT_NAME  # cls_names[cls]
                                d_info.category = cls
                                d_info.sight_angle = [(xmin+w/2.-0.5)*aos_x, (ymin+h/2.-0.5)*aos_y]
                                d_info.pixel_position = [pixel_cx, pixel_cy]
                
                                if OBJECT_HEIGHT > 0:
                                    depth = (OBJECT_HEIGHT*camera_matrix[1][1]) / (h*image_height)
                                    d_info.position = [math.tan(d_info.sight_angle[0])*depth, math.tan(d_info.sight_angle[1])*depth, depth]
                
                                m_info.detection_infos.append(d_info)
                                m_info.num_objs += 1

            if frame_id != last_fid:
                pub.publish(m_info)
                m_info = MultiDetectionInfo()
                m_info.num_objs = 0

            last_fid = frame_id
            # print("{:.3f}, {:.3f}, {:.3f}, {:.3f}".format(ex, ey ,ess, speed))
            # py_array = [ex * qx / 57.3, ey * qy / 57.3, ess, dt, lock_stat, prop]
            # ros_array = Float32MultiArray(data=py_array)
            # pose = Pose(Point(ex, ey ,ess), Quaternion(speed, dt, lock_stat, 0.))
            # pub.publish(ros_array)

    rate.sleep()

