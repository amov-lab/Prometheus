/*
 * @Author: Eason Yi
 * @Date: 2022-03-22 11:03:27
 * @LastEditTime: 2022-03-22 17:19:46
 * @LastEditors: Please set LastEditors
 * @Description: prometheus自主起飞demo
 * @FilePath: /Prometheus/Modules/tutorial_demo/takeoff/takeoff.cpp
 */
#include <ros/ros.h>
#include <iostream>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>

#include "math_utils.h"
#include "printf_utils.h"
using namespace std;

#define NODE_NAME "takeoff"

prometheus_msgs::UAVCommand uav_command;             // 发送指令到无人机
int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    // 默认使用1号机进行实验
    string uav_name = "/uav" + std::to_string(1); 
    //【订阅】无人机状态
    // ros::Subscriber uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>(uav_name  + "/prometheus/state", 10, uav_state_cb);
    //【发布】发送给控制模块
    ros::Publisher uav_command_pub = nh.advertise<prometheus_msgs::UAVCommand>(uav_name + "/prometheus/command", 1);
    while(ros::ok)
    {
        // 调用Init_Pos_Hover接口，高度设置在launch文件中修改
        // ~/Prometheus/Modules/uav_control/launch/uav_control_main.launch
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;   
        uav_command_pub.publish(uav_command);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
