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
#include <std_msgs/Float32.h>
#include <prometheus_msgs/ControlCommand.h>
#include <nav_msgs/Odometry.h>
using namespace std;
using namespace Eigen;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//---------------------------------------Drone---------------------------------------------  
float drone_height;   
float drone_yaw;
float start_point_x,start_point_y,start_point_z;
float velocity;
float k_p;
//---------------------------------------Vision---------------------------------------------
std_msgs::Float32 color_line_angle;         

//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vision_cb(const std_msgs::Float32::ConstPtr &msg)
{
    color_line_angle = *msg;
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
    ros::Subscriber vision_sub = nh.subscribe<std_msgs::Float32>("/prometheus/vision/color_line_angle", 10, vision_cb);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    nh.param<float>("velocity", velocity, 0.1);
    nh.param<float>("start_point_x", start_point_x, 0.0);
    nh.param<float>("start_point_y", start_point_y, 0.0);
    nh.param<float>("start_point_z", start_point_z, 2.0);

    nh.param<float>("k_p", k_p, 0.1);

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
    Command_Now.Command_ID = 1;
    while( drone_height < 0.3)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
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

        Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::BODY_FRAME;
        Command_Now.Reference_State.velocity_ref[0]     = velocity;
        Command_Now.Reference_State.velocity_ref[1]     = 0.0;
        Command_Now.Reference_State.position_ref[2]     = start_point_z;
        Command_Now.Reference_State.yaw_ref             = k_p * color_line_angle.data * 0.05;

        
        //Publish
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        //command_pub.publish(Command_Now);
        printf_result();
        rate.sleep();

    }

    return 0;

}

void printf_result()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>Color Line Following Mission<<<<<<<<<<<<<<<<<<<< "<< endl;
    
    cout << "color_line_angle: " << color_line_angle.data <<endl;
    cout << "yaw_ref: " << Command_Now.Reference_State.yaw_ref/3.1415926 *180 << " [deg] "<<endl;
}
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "velocity      : "<< velocity << endl;
    cout << "start_point_x : "<< start_point_x << endl;
    cout << "start_point_y : "<< start_point_y << endl;
    cout << "start_point_z : "<< start_point_z << endl;
    cout << "k_p           : "<< k_p << endl;
    
}

