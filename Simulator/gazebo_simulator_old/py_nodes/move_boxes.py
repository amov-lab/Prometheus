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
    black_box = ModelState()
    black_box.model_name = 'black_box'
    yellow_box = ModelState()
    yellow_box.model_name = 'yellow_box'
    blue_box = ModelState()
    blue_box.model_name = 'blue_box'
    red_box = ModelState()
    red_box.model_name = 'red_box'
    rate = rospy.Rate(100)
    time = 0.0
    while not rospy.is_shutdown():
        time = time + 0.002
        black_box.pose.position.x = 4.5*math.cos(3*time)+0.5
        black_box.pose.position.y = 4.5*math.sin(3*time)
        black_box.pose.position.z = 0.01
        

        yellow_box.pose.position.x = 4*math.cos(time) * math.cos(2*time)
        yellow_box.pose.position.y = 4*math.sin(time) * math.cos(2*time)+1
        yellow_box.pose.position.z = 0.01
        

        blue_box.pose.position.x = 4*math.cos(time) * math.cos(3*time)
        blue_box.pose.position.y = 4*math.sin(time) * math.cos(3*time)+1
        blue_box.pose.position.z = 0.01

        red_box.pose.position.x = 2*math.cos(time) * math.cos(4*time)+1
        red_box.pose.position.y = 4*math.sin(time) * math.cos(4*time)
        red_box.pose.position.z = 0.05
        pub.publish(blue_box)
        pub.publish(black_box)
        pub.publish(yellow_box)
        pub.publish(red_box)
        rate.sleep()

if __name__ == '__main__':
      rospy.init_node('box_pose_publisher')
      try:
          pose_publisher()
      except rospy.ROSInterruptException:
          pass
