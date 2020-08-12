//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>

//topic 头文件
#include <geometry_msgs/Point.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/MultiDetectionInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include "message_utils.h"

using namespace std;

#define MIN_DIS 0.1
#define FLY_HEIGHT 1.0
# define NODE_NAME "number_detection"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::MultiDetectionInfo Detection_info;
float size_Detection_info;
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
ros::Publisher command_pub;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void num_det_cb(const prometheus_msgs::MultiDetectionInfo::ConstPtr& msg)
{
    Detection_info = *msg;
    size_Detection_info = Detection_info.num_objs;
}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "number_detection");
    ros::NodeHandle nh("~");
    
    ros::Subscriber num_det_sub = nh.subscribe<prometheus_msgs::MultiDetectionInfo>("/prometheus/object_detection/num_det", 10, num_det_cb);

    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(3);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    int start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Number Tracking Mission<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to takeoff the drone..."<<endl;
        cin >> start_flag;
    }

    // 起飞
    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;
    while( _DroneState.position[2] < 0.3)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = 0.0;
        Command_Now.Reference_State.position_ref[1]     = 0.0;
        Command_Now.Reference_State.position_ref[2]     = 1.5;
        Command_Now.Reference_State.yaw_ref             = 0.5 * M_PI;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    while (ros::ok())
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Number Tracking Mission<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
        cout << "Total_Num:          " << size_Detection_info <<  endl;

        //回调
        ros::spinOnce();

        for(int i=0;i<10;i++)
        {
            cout << "Num:          " << i <<  endl;
            
            //判断是否识别到
            int idx = -1;
            for (int j=0;j<size_Detection_info;j++)
            {
                if(Detection_info.detection_infos[j].category == i)
                {
                    idx = j;
                    break;
                }
            }

            if(idx != -1)
            {
                cout << "relative_pos: " << Detection_info.detection_infos[idx].position[0] << " [m] "<< Detection_info.detection_infos[idx].position[1] << " [m] "<< Detection_info.detection_infos[idx].position[2] << " [m] "<<endl;
            }else{
                cout << "Not detected!!!" <<  endl;
            }
        }

        ros::Duration(1.0).sleep();
    }

    return 0;

}

