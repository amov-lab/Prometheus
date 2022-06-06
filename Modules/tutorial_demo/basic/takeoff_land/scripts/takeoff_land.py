#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 例程简介: 讲解如何调用uav_control的接口实现无人机ENU坐标系下的位置控制
# 效果说明: 无人机起飞后悬停30秒,然后降落
# 备注:该例程仅支持Prometheus仿真,真机测试需要熟练掌握相关接口的定义后以及真机适配修改后使用
from math import fabs
from turtle import position
import ros
import rospy
from prometheus_msgs.msg import UAVCommand, UAVControlState, UAVState

# 创建无人机相关数据变量
uav_control_state_sv = UAVControlState()
uav_command_pv = UAVCommand()
uav_state_sv = UAVState()

# 无人机状态回调函数
def uavStateCb(msg):
    global uav_state_sv
    uav_state_sv = msg

# 无人机控制状态回调函数
def uavControlStateCb(msg):
    global uav_control_state_sv
    uav_control_state_sv = msg

def main():
    # ROS初始化,设定节点名
    rospy.init_node('takeoff_land_py',anonymous=True)
    # 创建命令发布标志位,命令发布则为true;初始化为false
    cmd_pub_flag = False
    # 创建无人机控制命令发布者
    UavCommandPb = rospy.Publisher("/uav1/prometheus/command", UAVCommand, queue_size =10)
    # 创建无人机控制状态命令订阅者
    rospy.Subscriber("/uav1/prometheus/control_state", UAVControlState, uavControlStateCb)
    # 创建无人机状态命令订阅者
    rospy.Subscriber("/uav1/prometheus/state", UAVState, uavStateCb)
    # 循环频率设置为10HZ
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 检测无人机是否处于[COMMAND_CONTROL]模式
        if uav_control_state_sv.control_state == UAVControlState.COMMAND_CONTROL:
            # 检测控制命令是否发布,没有发布则进行命令的发布
            if not cmd_pub_flag:
                # 时间戳
                uav_command_pv.header.stamp = rospy.Time.now()
                # 坐标系
                uav_command_pv.header.frame_id = 'ENU'
                # Init_Pos_Hover初始位置悬停,可在uav_control_indoor.yaml或uav_control_outdoor.yaml文件设置无人机悬停高度
                uav_command_pv.Agent_CMD = 1
                # 发布的命令ID,每发一次,该ID加1
                uav_command_pv.Command_ID = 1
                # 发布起飞命令
                UavCommandPb.publish(uav_command_pv)
                rate.sleep()
                # 命令发布标志位置为true
                cmd_pub_flag = True
                # 打印无人机起飞相关信息
                rospy.loginfo("Takeoff_height: %d", rospy.get_param('/uav_control_main_1/control/Takeoff_height'))
            else:
                # 当无人机距离高度目标值±0.1米范围内时认为起飞完成并等待30秒后降落
                if fabs(uav_state_sv.position[2] - rospy.get_param('/uav_control_main_1/control/Takeoff_height')) <= 0.1:
                    print(" UAV takeoff successfully and landed after 30 seconds")
                    rospy.sleep(30)
                    # 时间戳
                    uav_command_pv.header.stamp = rospy.Time.now()
                    # 坐标系
                    uav_command_pv.header.frame_id = "ENU"
                    # Land降落,从当前位置降落至地面并自动上锁
                    uav_command_pv.Agent_CMD = 3
                    # 发布的命令ID加1
                    uav_command_pv.Command_ID += 1
                    # 发布降落命令
                    UavCommandPb.publish(uav_command_pv)
                    # 打印降落相关信息
                    rospy.loginfo("UAV Land")
                    rospy.loginfo("[takeoff & land tutorial_demo completed]")
                    # 任务结束,关闭该节点
                    rospy.signal_shutdown("shutdown time")
                else:
                    # 打印当前无人机高度信息
                    rospy.loginfo("UAV height : %f [m]", uav_state_sv.position[2])
                    rospy.sleep(1)
        else:
            # 在控制命令发布后,但无人机未结束任务的情况下,此时无人机未处于[COMMAND_CONTROL]控制状态,认为无人机出现意外情况,任务中止
            if cmd_pub_flag:
                rospy.logfatal(" Unknown error! [takeoff & land] tutorial_demo aborted")
            # 命令未发布,等待无人机进入[COMMAND_CONTROL]状态
            else:
                rospy.logwarn(" Wait for UAV to enter [COMMAND_CONTROL] MODE ")
                rospy.sleep(2)
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass