
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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Path.h>
using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::PositionReference pos_cmd;
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
ros::Publisher command_pub;
int flag_get_cmd = 0;
geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped drone_pos;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void pos_cmd_cb(const prometheus_msgs::PositionReference::ConstPtr& msg)
{
    flag_get_cmd = 1;
    pos_cmd = *msg;

    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.Reference_State = pos_cmd;

    command_pub.publish(Command_Now);
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
}



void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "fast_planner");
    ros::NodeHandle nh("~");
    ros::Subscriber pos_cmd_sub =nh.subscribe<prometheus_msgs::PositionReference>("/planning/position_cmd", 50, pos_cmd_cb);
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/planning/goal", 10,goal_cb);
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

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

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(4);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    
    while (ros::ok())
    {
        //回调
        ros::spinOnce();
        if( flag_get_cmd == 0)
        {

        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Hold;
        Command_Now.Command_ID                          = 1;

        command_pub.publish(Command_Now);
        cout << "waiting for trajectory" << endl;
        }
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Planning<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "drone_pos: " << _DroneState.position[0] << " [m] "<< _DroneState.position[1] << " [m] "<< _DroneState.position[2] << " [m] "<<endl;
        cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;


        rate.sleep();
    }

    return 0;

}
