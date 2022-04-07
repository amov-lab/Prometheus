#! /usr/bin/env python
# -*- coding: utf-8 -*-
#author : Colin Lee
#email  : lcyfly1@163.com
#description  :  control qrcode movement 
import rospy
import math
from gazebo_msgs.msg import ModelState

move_type = 1


def pose_publisher_line():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'car_landing_pad'
    rate = rospy.Rate(100)
    linear_vel = 1.0
    time = 0.0
    while not rospy.is_shutdown():
        pos = -5 + time * linear_vel
        time = time + 0.01
        if pos > 30.0:
            time = 0
        else:
            pass
        
        pose_msg.pose.position.x = pos
        pose_msg.pose.position.y = -2.0
        pose_msg.pose.position.z = 0.01
        pub.publish(pose_msg)
        print('Pos_x :',pose_msg.pose.position.x)
        print('Pos_y :',pose_msg.pose.position.y)
        rate.sleep()


def pose_publisher_circle():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'car_landing_pad'
    rate = rospy.Rate(100)
    linear_vel = 0.5
    circle_radius = 3.0
    omega = math.fabs(linear_vel / circle_radius)
    time = 0.0
    while not rospy.is_shutdown():
        angle = time * omega
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        time = time + 0.01
        pose_msg.pose.position.x = circle_radius*cos_angle
        pose_msg.pose.position.y = circle_radius*sin_angle
        pose_msg.pose.position.z = 0.01
        pub.publish(pose_msg)
        print('Pos_x :',pose_msg.pose.position.x)
        print('Pos_y :',pose_msg.pose.position.y)
        rate.sleep()

if __name__ == '__main__':
      rospy.init_node('car_pose_publisher')
      try:
          if move_type == 0:
            pose_publisher_circle()
          elif move_type == 1:
            pose_publisher_line()
          
      except rospy.ROSInterruptException:
          pass
