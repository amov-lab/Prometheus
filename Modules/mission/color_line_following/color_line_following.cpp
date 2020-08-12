/***************************************************************************************************************************
 * color_line_following.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2020.4.6
 *
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>
#include <tf/transform_datatypes.h>
//topic 头文件
#include <prometheus_msgs/DroneState.h>
#include <geometry_msgs/Pose.h>
#include <prometheus_msgs/ControlCommand.h>
#include <nav_msgs/Odometry.h>
#include "message_utils.h"

using namespace std;
using namespace Eigen;
#define FOLLOWING_VEL 0.5
#define FOLLOWING_KP 2.0
# define NODE_NAME "color_line_following"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//---------------------------------------Drone---------------------------------------------  
float drone_height;   
float drone_yaw;
float start_point_x,start_point_y,start_point_z;
//---------------------------------------Vision---------------------------------------------
int flag_detection;
float error_body_y;
float yaw_sp;
//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void color_line_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    error_body_y = - tan(msg->position.x) * start_point_z;
    flag_detection = msg->position.y;

    float x1 = msg->orientation.w;
    float y1 = msg->orientation.x;
    float x2 = msg->orientation.y;
    float y2 = msg->orientation.z;

    float next_desired_yaw = - atan2(y2 - y1, x2 - x1);

    yaw_sp = (0.7*yaw_sp + 0.3*next_desired_yaw);
}
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    drone_yaw = msg->attitude[2];
    drone_height = msg->position[2];
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_line_following");
    ros::NodeHandle nh("~");

    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    //【订阅】所识别线与相机中心轴的偏移角度
    ros::Subscriber color_line_sub = nh.subscribe<geometry_msgs::Pose>("/prometheus/object_detection/color_line_angle", 10, color_line_cb);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    nh.param<float>("start_point_x", start_point_x, 0.0);
    nh.param<float>("start_point_y", start_point_y, 0.0);
    nh.param<float>("start_point_z", start_point_z, 2.0);

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    printf_param();
    // Waiting for input
    int start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>Color Line Following Mission<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to start... "<<endl;
        cin >> start_flag;
    }

    // 起飞
    cout<<"[color_line_following]: "<<"Takeoff to predefined position."<<endl;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Takeoff to predefined position.");
    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;
    while( drone_height < 0.3)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = start_point_x;
        Command_Now.Reference_State.position_ref[1]     = start_point_y;
        Command_Now.Reference_State.position_ref[2]     = start_point_z;
        Command_Now.Reference_State.yaw_ref             = 0.0;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    ros::Duration(5.0).sleep();

    // 先读取一些飞控的数据
    for(int i=0;i<10;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::MIX_FRAME;
        Command_Now.Reference_State.velocity_ref[0]     = FOLLOWING_VEL;
        Command_Now.Reference_State.velocity_ref[1]     = FOLLOWING_KP * error_body_y;
        Command_Now.Reference_State.position_ref[2]     = start_point_z;
        Command_Now.Reference_State.yaw_ref             = yaw_sp;
        command_pub.publish(Command_Now);
        printf_result();
        rate.sleep();

    }

    return 0;

}

void printf_result()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>Color Line Following Mission<<<<<<<<<<<<<<<<<<<< "<< endl;
    cout << "error_body_y: " << error_body_y << " [m] "<<endl;
    cout << "yaw_sp: " << yaw_sp/3.1415926 *180 << " [deg] "<<endl;
}
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "start_point_x : "<< start_point_x << endl;
    cout << "start_point_y : "<< start_point_y << endl;
    cout << "start_point_z : "<< start_point_z << endl;
    cout << "FOLLOWING_KP           : "<< FOLLOWING_KP << endl;
    
}

