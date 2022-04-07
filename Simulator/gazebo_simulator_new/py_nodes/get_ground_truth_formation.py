#! /usr/bin/env python
# -*- coding: utf-8 -*-
#author : Yuhua Qi
#email  : fatmoonqyp@126.com
#description  :  get_ground_truth_formation
import rospy
import math
import yaml
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
import sys
import tf


def get_odom(id):
    try:
        handle = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
        response = handle('P300_uav'+str(id),'ground_plane')
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def pose_publisher(uav_num):

    while True:
        for i in range(uav_num):
            odom= get_odom(i+1)
            multi_local_pose[i].header.stamp = rospy.Time.now()
            multi_local_pose[i].pose = odom.pose
            multi_pose_pub[i].publish(multi_local_pose[i])
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('formation_ground_pose_publisher')

    uav_num = rospy.get_param('~uav_num',5)

    multi_pose_pub = [None]*uav_num
    multi_local_pose = [None]*uav_num
    for i in range(uav_num):
        multi_pose_pub[i] = rospy.Publisher("/uav"+str(i+1)+"/mavros/vision_pose/pose", PoseStamped, queue_size=2)
        multi_local_pose[i] = PoseStamped()
        multi_local_pose[i].header.frame_id = 'map'
        print('UAV'+str(i+1)+":OK")
    rate = rospy.Rate(100)

    try:
        pose_publisher(uav_num)
    except rospy.ROSInterruptException:
        pass
