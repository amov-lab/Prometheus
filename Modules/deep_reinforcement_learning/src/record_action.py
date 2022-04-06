#! /usr/bin/env python
# -*- coding: utf-8 -*-

## version 2: 
## 1, navigate the robot using a constant heading angle
## 2, add the ddpg neural network
## 3, 24 laser data and just heading
## 4, added potential collisions



## Command:
## roslaunch turtlebot_iros turtlebot_world.launch world_file:='/home/hanlin/catkin_ws/src/turtlebot/turtlebot_iros/modified_world.world'
## source ~/iros_env/bin/activate
## rosrun turtlebot_iros ddpg_turtlebot.py

import rospy
import rospkg
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
from math import radians, copysign, sqrt, pow, pi, atan2, asin
from tf.transformations import euler_from_quaternion

import threading
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState 

from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
#from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time


import numpy as np
import math
import random
from collections import deque
from std_srvs.srv import Empty
import scipy.io as sio


class InfoGetter(object):
    # 多线程读取msg
    def __init__(self):
        #event that will block until the info is received
        self._event = threading.Event()
        #attribute for storing the rx'd message
        self._msg = None

    def __call__(self, msg):
        #Uses __call__ so the object itself acts as the callback
        #save the data, trigger the event
        self._msg = msg
        self._event.set()

    def get_msg(self, timeout=None):
        """Blocks until the data is rx'd with optional timeout
        Returns the received message
        """
        self._event.wait(timeout)
        return self._msg


class GameState:

    def __init__(self):
        
        self.talker_node = rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(50) # ros频率是50hz
        print("init ...")
        self.laser_ig = InfoGetter()
        self.odom_ig = InfoGetter()
        self.collect_ig = InfoGetter()
        self.action_ig = InfoGetter()

        self.laser_info = rospy.Subscriber("/scan_output", LaserScan, self.laser_ig)
        self.odom_info = rospy.Subscriber("/uav1/fake_odom", Odometry, self.odom_ig)
        self.start_info = rospy.Subscriber("/collect", PoseStamped, self.collect_ig)
        self.action_info = rospy.Subscriber("/uav1/ego_action", Twist, self.action_ig)

        self.position = Point()

        # 碰撞标志 crush default value
        self.crash_indicator = 0
        # 雷达reward
        self.laser_reward = 0

        # observation_space and action_space
        self.state_num = 28 #685               
        self.action_num = 2
        self.observation_space = np.empty(self.state_num)
        self.action_space = np.empty(self.action_num)

        self.memory = deque(maxlen=4000000)

        self.return_action = np.empty(self.action_num)
        self.return_action = self.return_action.reshape((1, self.action_space.shape[0]))

    def turtlebot_is_crashed(self, range_limit):
        # 计算碰撞奖励函数(使用最新的雷达数据)
        self.laser_crashed_value = 0
        self.laser_crashed_reward = 0

        for i in range(len(self.laser_values)):
            # 进入警戒范围
            if (self.laser_values[i] < 2*range_limit):
                self.laser_crashed_reward = -80
            # 碰撞
            if (self.laser_values[i] < range_limit):
                self.laser_crashed_value = 1
                self.laser_crashed_reward = -200
                # self.reset()
                # time.sleep(1)
                break
        return self.laser_crashed_reward

    def read_state(self):
        # 读取当前状态,返回state和速度action
        # 全局变量改变:目标:self.target  当前位置:self.position  当前速度:self.return_action  雷达:self.laser_values

        # 读取目标点
        collect_flag = self.collect_ig.get_msg()
        self.target_y = collect_flag.pose.position.y
        self.target_x = collect_flag.pose.position.x

        # 读取当前位置
        odom = self.odom_ig.get_msg()
        self.position = (odom.pose.pose.position.x,odom.pose.pose.position.y)
        self.rotation = odom.twist.twist.angular.z
        turtlebot_x = self.position[0]
        turtlebot_y = self.position[1]
        angle_turtlebot = self.rotation

        # 读取雷达数据并归一化
        laser_msg = self.laser_ig.get_msg()
        self.laser_values = laser_msg.ranges
        self.normalized_laser = [(x)/5 for x in (laser_msg.ranges)]

        # 读取当前速度存入self.return_action
        # 读取当前线速度和角速度(全局变量)
        action_value = self.action_ig.get_msg()

        action_now = []
        action_now.append(action_value.angular.z)
        action_now.append(action_value.linear.x)
        action_now = np.array(action_now)

        self.return_action[0][0] = action_value.angular.z
        self.return_action[0][1] = action_value.linear.x

        # 与目标点的距离
        current_distance_turtlebot_target = math.sqrt((self.target_x - turtlebot_x)**2 + (self.target_y - turtlebot_y)**2)

        # 算质点与目标点角度
        angle_turtlebot_target = atan2(self.target_y - turtlebot_y, self.target_x- turtlebot_x)

        if angle_turtlebot < 0:
            angle_turtlebot = angle_turtlebot + 2*math.pi

        if angle_turtlebot_target < 0:
            angle_turtlebot_target = angle_turtlebot_target + 2*math.pi
 
        # 添加yaw角的与目标点角度
        angle_diff = angle_turtlebot_target - angle_turtlebot
        if angle_diff < -math.pi:
            angle_diff = angle_diff + 2*math.pi
        if angle_diff > math.pi:
            angle_diff = angle_diff - 2*math.pi

        # 合并状态为array:归一化雷达,与目标距离.与目标角度,线速度,角速度
        state = np.append(self.normalized_laser, current_distance_turtlebot_target)
        state = np.append(state, angle_diff)
        state = np.append(state, self.return_action[0][1])
        state = np.append(state, self.return_action[0][0])
        state = state.reshape(1, self.state_num)

        return state, action_now


    def read_game_step(self, time_step, linear_x, angular_z):
        print "current_linear_x: " , linear_x
        print "current_aangular_z: " , angular_z

        # 记录前一时刻位置
        turtlebot_x_previous = self.position[0]
        turtlebot_y_previous = self.position[1]

        # 计时超过0.1s后读取下一时刻状态
        start_time = time.time()
        record_time = start_time
        record_time_step = 0
        while (record_time_step < time_step):
            self.rate.sleep()
            record_time = time.time()
            record_time_step = record_time - start_time

        # 以下开始记录新一时刻状态
        # print "current_action1.25: " , current_action

        # 读取新一时刻状态:包含归一化雷达,与目标距离.与目标角度,线速度,角速度
        # 全局变量改变:目标:self.target  当前位置:self.position  当前速度:self.return_action  雷达:self.laser_values,normalized_laser       
        state_new, action_new = self.read_state()
        # print "current_action1.3: " , current_action
        # 记录新时刻位置
        turtlebot_x = self.position[0]
        turtlebot_y = self.position[1]

        # 计算reward
        # 计算与目标点之间缩短的距离
        distance_turtlebot_target_previous = math.sqrt((self.target_x - turtlebot_x_previous)**2 + (self.target_y - turtlebot_y_previous)**2)
        distance_turtlebot_target = math.sqrt((self.target_x - turtlebot_x)**2 + (self.target_y - turtlebot_y)**2)
        distance_reward = distance_turtlebot_target_previous - distance_turtlebot_target
        # 计算碰撞奖励 (更新self.laser_crashed_value, self.laser_crashed_reward)
        self.laser_crashed_reward = self.turtlebot_is_crashed(range_limit=0.25)  # 是否碰撞或进入警戒区 
        self.laser_reward = sum(self.normalized_laser)-24  # 雷达距离判断周围所有障碍物距离
        self.collision_reward = self.laser_crashed_reward + self.laser_reward
        # 过度转向惩罚
        self.angular_punish_reward = 0
        if angular_z > 0.8:
            self.angular_punish_reward = -1
        if angular_z < -0.8:
            self.angular_punish_reward = -1

        # 走太慢惩罚
        self.linear_punish_reward = 0
        if linear_x < 0.2:
            self.linear_punish_reward = -2
        # 到达奖励
        self.arrive_reward = 0
        if distance_turtlebot_target<1:
            self.arrive_reward = 100

        reward  = distance_reward*(5/time_step)*1.2*7 + self.arrive_reward + self.collision_reward + self.angular_punish_reward + self.linear_punish_reward

        return reward, state_new, self.laser_crashed_value

    def remember(self, cur_state, action, reward, new_state, done):
        # 保存数据
        # 数据变成array并指定格式
        cur_state = cur_state.reshape(28)
        new_state = new_state.reshape(28)
        action = action.reshape(2)

        print "current_cur_state: " , cur_state

        print "current_action3: " , action

        print "current_new_state: " , new_state
        self.array_reward = np.array(reward)
        self.array_reward = self.array_reward.reshape(1)  
        done = np.array(done)
        done = done.reshape(1)
        # 拼接到memory_pack上
        self.memory_pack = np.concatenate((cur_state, action))
        self.memory_pack = np.concatenate((self.memory_pack, self.array_reward))
        self.memory_pack = np.concatenate((self.memory_pack, new_state))
        self.memory_pack = np.concatenate((self.memory_pack, done))
        self.memory.append(self.memory_pack)
        # print("self.memory length is %s", len(self.memory))
        # 保存
        if len(self.memory) % 10 == 0:
            sio.savemat('human_data_my.mat', {'data': self.memory}, True, '5', False, False, 'row')


if __name__ == '__main__':
    try:
        game_state = GameState()
        collect_num = 0
        #game_state.reset()
        while True:
            # 读取是否收到目标点
            collect_flag = game_state.collect_ig.get_msg()
            # while((game_state.collect_ig.get_msg()).pose.position.z == 1):
            if (collect_flag.pose.position.z == 1):
                collect_num += 1
                print "collecting: " , collect_num
                # 读取当前状态和速度
                current_state, current_action = game_state.read_state()  
                # 变形
                current_state = current_state.reshape((1, game_state.observation_space.shape[0]))
                current_action = current_action.reshape((1, game_state.action_space.shape[0]))
                print "current_action1: " , current_action
                # 读取动作0.1s后的新状态并以此计算reward
                reward, new_state, crashed_value = game_state.read_game_step(0.1, current_action[0][1], current_action[0][0])
                print "current_action2: " , current_action
                # 存经验池
                game_state.remember(current_state, current_action, reward, new_state, crashed_value)
                time.sleep(1.0)
            else :
                print "not collecting ..."
                time.sleep(1.0)
                

    except rospy.ROSInterruptException:
        pass


