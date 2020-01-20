
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
#include <nav_msgs/Odometry.h>
using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

prometheus_msgs::PositionReference pos_cmd;
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
ros::Publisher odom_pub;

prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void pos_cmd_cb(const prometheus_msgs::PositionReference::ConstPtr& msg)
{
    pos_cmd = *msg;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
    nav_msgs::Odometry Odom_Now;
    Odom_Now.header.stamp = ros::Time::now();
    Odom_Now.header.frame_id = "world";

    Odom_Now.pose.pose.position.x = _DroneState.position[0];
    Odom_Now.pose.pose.position.y = _DroneState.position[1];
    Odom_Now.pose.pose.position.z = _DroneState.position[2];

//    Odom_Now.pose.pose.orientation = geometry_msgs::Quaternion(_DroneState.attitude[2],_DroneState.attitude[1], _DroneState.attitude[0]); // yaw, pitch, roll
    Odom_Now.pose.pose.orientation = _DroneState.attitude_q;
    Odom_Now.twist.twist.linear.x = _DroneState.velocity[0];
    Odom_Now.twist.twist.linear.y = _DroneState.velocity[1];
    Odom_Now.twist.twist.linear.z = _DroneState.velocity[2];
    odom_pub.publish(Odom_Now);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh("~");
    ros::Subscriber pos_cmd_sub =nh.subscribe<prometheus_msgs::PositionReference>("planning/position_cmd", 50, pos_cmd_cb);

    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);
    //【发布】无人机odometry
    odom_pub = nh.advertise<nav_msgs::Odometry>("/planning/odom_world", 10);

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
