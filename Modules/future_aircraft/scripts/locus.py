#!/usr/bin/env python3
import rospy
import math
import time as t
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse


client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
client.wait_for_service()

def init_pose():
    req = SetModelStateRequest()
    req.model_state.model_name = "future_aircraft_car"
    req.model_state.pose.position.x = 0
    req.model_state.pose.position.y = 0
    req.model_state.pose.position.z = 0
    client.call(req)

landing_pad = None
def get_pos(info: ModelStates):
    global landing_pad
    for i , name in enumerate(info.name):
        if name == "future_aircraft_car":
            landing_pad = info.pose[i].position
            # print(landing_pad)
            return


def pose_publisher_circle():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'landing_pad'
    pose_msg.pose.position.x = 0
    pose_msg.pose.position.y = 0
    pose_msg.pose.position.z = 0

    rate = rospy.Rate(30)
    pub = rospy.Publisher('/wheeltec/cmd_vel', Twist, queue_size=10)
    pose_msg = Twist()
    rate = rospy.Rate(60)
    circle_radius = 1.5
    linear_vel = 1
    omega = math.fabs(linear_vel / circle_radius)
    # cycle = math.pi * 2 / omega
    past_time = t.time()
    while not rospy.is_shutdown():
        pose_msg.linear.x = linear_vel
        pose_msg.angular.z = omega
        if landing_pad != None and math.sqrt(landing_pad.x ** 2 + landing_pad.y**2) < 0.3 and t.time() - past_time > 1:
            pub.publish(pose_msg)
            init_pose()
            past_time = t.time()
            omega = -omega
        pub.publish(pose_msg)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('locus_8')
    rospy.Subscriber("/gazebo/model_states", ModelStates, get_pos)
    init_pose()
    pose_publisher_circle()
