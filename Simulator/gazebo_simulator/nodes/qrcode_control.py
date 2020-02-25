#! /usr/bin/env python
# -*- coding: utf-8 -*-
#author : Colin Lee
#email  : lcyfly1@163.com
#description  :  control qrcode movement 
import rospy
import math
from gazebo_msgs.msg import ModelState

def pose_publisher():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'qrcode_cube'
    rate = rospy.Rate(30)
    radius = 0.5
    theta = 0.
    while not rospy.is_shutdown():
           theta += (15./30)%2000
           pose_msg.pose.position.x = radius*math.cos(math.radians(theta))
           pose_msg.pose.position.y = radius*math.sin(math.radians(theta))
           pose_msg.pose.position.z = 1
           pub.publish(pose_msg)
           print('Pos_x :',pose_msg.pose.position.x)
           print('Pos_y :',pose_msg.pose.position.y)
           rate.sleep()

if __name__ == '__main__':
      rospy.init_node('qrcode_pose_publisher')
      try:
          pose_publisher()
      except rospy.ROSInterruptException:
          pass
