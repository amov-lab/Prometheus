#! /usr/bin/env python
# -*- coding: utf-8 -*-
#author : Yuhua Qi
#email  : fatmoonqyp@126.com
#description  :  get_ground_truth
import rospy
import math
import yaml
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
import sys
import tf


def get_odom(uav_name):
    try:
        handle = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
        response = handle(uav_name,'ground_plane')
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def pose_publisher(uav_name):

    while True:
        odom= get_odom(uav_name)
        ground_truth.header.stamp = rospy.Time.now()
        ground_truth.pose = odom.pose
        ground_truth_pub.publish(ground_truth)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ground_pose_publisher')

    uav_name = rospy.get_param('~uav_name',"P300_basic")

    ground_truth_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=2)
    ground_truth = PoseStamped()
    ground_truth.header.frame_id = 'map'
    print('uav_name:'+uav_name)

    rate = rospy.Rate(100)

    try:
        pose_publisher(uav_name)
    except rospy.ROSInterruptException:
        pass
