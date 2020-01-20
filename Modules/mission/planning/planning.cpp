
//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>

//topic 头文件
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include "prometheus_msgs/PositionReference.h"
using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

prometheus_msgs::PositionReference pos_cmd;
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void pos_cmd_cb(const prometheus_msgs::PositionReference::ConstPtr& msg)
{
    pos_cmd = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh("~");
    ros::Subscriber pos_cmd_sub =nh.subscribe<prometheus_msgs::PositionReference>("/position_cmd", 50, pos_cmd_cb);

    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 频率 [20Hz]
    ros::Rate rate(20.0);

    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID                          = 0;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0;
    Command_Now.Reference_State.position_ref[1]     = 0;
    Command_Now.Reference_State.position_ref[2]     = 0;
    Command_Now.Reference_State.velocity_ref[0]     = 0;
    Command_Now.Reference_State.velocity_ref[1]     = 0;
    Command_Now.Reference_State.velocity_ref[2]     = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref             = 0;
    command_pub.publish(Command_Now);


    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.Reference_State   = pos_cmd;

        command_pub.publish(Command_Now);

        rate.sleep();
    }

    return 0;

}
