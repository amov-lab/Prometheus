/***************************************************************************************************************************
 * waypoint_tracking.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2020.1.12
 *
 * 说明: 航点追踪
***************************************************************************************************************************/

#include <ros/ros.h>
#include <iostream>
#include "mission_utils.h"
#include "message_utils.h"
#include <std_msgs/Bool.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
using namespace std;
 
# define TIME_OUT 20.0
# define THRES_DISTANCE 0.15
# define NODE_NAME "waypoint_tracking"
bool sim_mode;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::ControlCommand Command_Now;
prometheus_msgs::DroneState _DroneState;                          //无人机状态量
Eigen::Vector3f drone_pos;

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    drone_pos[0] = _DroneState.position[0];
    drone_pos[1] = _DroneState.position[1];
    drone_pos[2] = _DroneState.position[2];
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_tracking");
    ros::NodeHandle nh("~");

    // 频率 [1hz]
    ros::Rate rate(1.0);

    //【订阅】无人机当前状态
    // 本话题来自根据需求自定px4_pos_estimator.cpp
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    ros::Time time_begin;
    float time_sec;
    float distance;
    Eigen::Vector3f point1;
    Eigen::Vector3f point2;
    Eigen::Vector3f point3;
    Eigen::Vector3f point4;
    Eigen::Vector3f point5;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // 仿真模式 - 区别在于是否自动切换offboard模式
    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<float>("point1_x", point1[0], 0.0);
    nh.param<float>("point1_y", point1[1], 0.0);
    nh.param<float>("point1_z", point1[2], 0.0);
    nh.param<float>("point2_x", point2[0], 0.0);
    nh.param<float>("point2_y", point2[1], 0.0);
    nh.param<float>("point2_z", point2[2], 0.0);
    nh.param<float>("point3_x", point3[0], 0.0);
    nh.param<float>("point3_y", point3[1], 0.0);
    nh.param<float>("point3_z", point3[2], 0.0);
    nh.param<float>("point4_x", point4[0], 0.0);
    nh.param<float>("point4_y", point4[1], 0.0);
    nh.param<float>("point4_z", point4[2], 0.0);
    nh.param<float>("point5_x", point5[0], 0.0);
    nh.param<float>("point5_y", point5[1], 0.0);
    nh.param<float>("point5_z", point5[2], 0.0);

    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "point1: " << "[ "<<point1[0]<< "," <<point1[1]<<","<<point1[2]<<" ]"<<endl;
    cout << "point2: " << "[ "<<point2[0]<< "," <<point2[1]<<","<<point2[2]<<" ]"<<endl;    
    cout << "point3: " << "[ "<<point3[0]<< "," <<point3[1]<<","<<point3[2]<<" ]"<<endl;
    cout << "point4: " << "[ "<<point4[0]<< "," <<point4[1]<<","<<point4[2]<<" ]"<<endl;
    cout << "point5: " << "[ "<<point5[0]<< "," <<point5[1]<<","<<point5[2]<<" ]"<<endl;

    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID                          = 0;
    Command_Now.source = NODE_NAME;
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

    if(sim_mode)
    {
        // Waiting for input
        int check_flag;
        cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
        cin >> check_flag;

        if(check_flag != 1)
        {
            return -1;
        }

        while(ros::ok() && _DroneState.mode != "OFFBOARD")
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
            Command_Now.Command_ID = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;
            Command_Now.Reference_State.yaw_ref = 999;
            move_pub.publish(Command_Now);   
            cout << "Switch to OFFBOARD and arm ..."<<endl;
            ros::Duration(2.0).sleep();
            ros::spinOnce();
        }
    }else
    {
        while(ros::ok() && _DroneState.mode != "OFFBOARD")
        {
            cout<<"[waypoint_tracking]: "<<"Please arm and switch to OFFBOARD mode."<<endl;
            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Please arm and switch to OFFBOARD mode.");
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }
    }

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主程序<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //takeoff
    cout<<"[waypoint_tracking]: "<<"Takeoff."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Takeoff;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

    move_pub.publish(Command_Now);

    sleep(10.0);
    
    //第一个目标点，左下角
    cout<<"[waypoint_tracking]: "<<"Moving to Point 1."<<endl;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Moving to Point 1.");
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = point1[0];
    Command_Now.Reference_State.position_ref[1] = point1[1];
    Command_Now.Reference_State.position_ref[2] = point1[2];
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    time_begin = ros::Time::now();
    distance = cal_distance(drone_pos,point1);
    time_sec = 0;
    while(distance > THRES_DISTANCE && time_sec < TIME_OUT)
    {
        ros::Time time_now = ros::Time::now();
        time_sec = time_now.sec-time_begin.sec;
        distance = cal_distance(drone_pos,point1);
        cout<<"[waypoint_tracking]: "<<"Distance to waypoint: "<<distance<< "[m], used " <<time_sec << "[s]."<<endl;
        ros::spinOnce();
        rate.sleep();
    }

        
    //第二个目标点，左上角
    cout<<"[waypoint_tracking]: "<<"Moving to Point 2."<<endl;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Moving to Point 2.");
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = point2[0];
    Command_Now.Reference_State.position_ref[1] = point2[1];
    Command_Now.Reference_State.position_ref[2] = point2[2];
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    time_begin = ros::Time::now();
    distance = cal_distance(drone_pos,point2);
    time_sec = 0;
    while(distance > THRES_DISTANCE && time_sec < TIME_OUT)
    {
        ros::Time time_now = ros::Time::now();
        time_sec = time_now.sec-time_begin.sec;
        distance = cal_distance(drone_pos,point2);
        cout<<"[waypoint_tracking]: "<<"Distance to waypoint: "<<distance<< "[m], used " <<time_sec << "[s]."<<endl;
        ros::spinOnce();
        rate.sleep();
    }

    //第三个目标点，右上角
    cout<<"[waypoint_tracking]: "<<"Moving to Point 3."<<endl;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Moving to Point 3.");
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = point3[0];
    Command_Now.Reference_State.position_ref[1] = point3[1];
    Command_Now.Reference_State.position_ref[2] = point3[2];
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    time_begin = ros::Time::now();
    distance = cal_distance(drone_pos,point3);
    time_sec = 0;
    while(distance > THRES_DISTANCE && time_sec < TIME_OUT)
    {
        ros::Time time_now = ros::Time::now();
        time_sec = time_now.sec-time_begin.sec;
        distance = cal_distance(drone_pos,point3);
        cout<<"[waypoint_tracking]: "<<"Distance to waypoint: "<<distance<< "[m], used " <<time_sec << "[s]."<<endl;
        ros::spinOnce();
        rate.sleep();
    }

    //第四个目标点，右下角
    cout<<"[waypoint_tracking]: "<<"Moving to Point 4."<<endl;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Moving to Point 4.");
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = point4[0];
    Command_Now.Reference_State.position_ref[1] = point4[1];
    Command_Now.Reference_State.position_ref[2] = point4[2];
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    time_begin = ros::Time::now();
    distance = cal_distance(drone_pos,point4);
    time_sec = 0;
    while(distance > THRES_DISTANCE && time_sec < TIME_OUT)
    {
        ros::Time time_now = ros::Time::now();
        time_sec = time_now.sec-time_begin.sec;
        distance = cal_distance(drone_pos,point4);
        cout<<"[waypoint_tracking]: "<<"Distance to waypoint: "<<distance<< "[m], used " <<time_sec << "[s]."<<endl;
        ros::spinOnce();
        rate.sleep();
    }

    //第五个目标点，回到起点
    cout<<"[waypoint_tracking]: "<<"Moving to Point 5."<<endl;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Moving to Point 5.");
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = point5[0];
    Command_Now.Reference_State.position_ref[1] = point5[1];
    Command_Now.Reference_State.position_ref[2] = point5[2];
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    time_begin = ros::Time::now();
    distance = cal_distance(drone_pos,point5);
    time_sec = 0;
    while(distance > THRES_DISTANCE && time_sec < TIME_OUT)
    {
        ros::Time time_now = ros::Time::now();
        time_sec = time_now.sec-time_begin.sec;
        distance = cal_distance(drone_pos,point5);
        cout<<"[waypoint_tracking]: "<<"Distance to waypoint: "<<distance<< "[m], used " <<time_sec << "[s]."<<endl;
        ros::spinOnce();
        rate.sleep();
    }

    //降落
    cout<<"[waypoint_tracking]: "<<"Landing."<<endl;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Landing.");
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Land;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    move_pub.publish(Command_Now);

    sleep(1.0);

    return 0;
}
