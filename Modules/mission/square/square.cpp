/***************************************************************************************************************************
 * square.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2020.1.12
 *
 * 说明: 正方形飞行示例程序
 *      1.
 *      2.
 *      3.
***************************************************************************************************************************/

#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Bool.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::ControlCommand Command_Now;
prometheus_msgs::DroneState _DroneState;                          //无人机状态量
//---------------------------------------正方形参数---------------------------------------------
float size_square;                  //正方形边长
float height_square;                //飞行高度
float sleep_time;

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "square");
    ros::NodeHandle nh("~");

    // 频率 [1hz]
    ros::Rate rate(1.0);

    //【订阅】无人机当前状态
    // 本话题来自根据需求自定px4_pos_estimator.cpp
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    nh.param<float>("size_square", size_square, 1.5);
    nh.param<float>("height_square", height_square, 1.0);
    nh.param<float>("sleep_time", sleep_time, 10.0);

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "size_square: "<<size_square<<"[m]"<<endl;
    cout << "height_square: "<<height_square<<"[m]"<<endl;
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    int i = 0;
    int comid = 0;

    // 无人机未解锁或者未进入offboard模式前，循环等待
    while(_DroneState.armed != true || _DroneState.mode != "OFFBOARD")
    {
        cout<<"[sqaure]: "<<"Please arm and switch to OFFBOARD mode."<<endl;
        ros::spinOnce();
        rate.sleep();
    }

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

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主程序<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //takeoff
    cout<<"[sqaure]: "<<"Takeoff."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = 0;
    Command_Now.Reference_State.position_ref[1] = 0;
    Command_Now.Reference_State.position_ref[2] = height_square;
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    sleep(sleep_time);
    
    //第一个目标点，左下角
    cout<<"[sqaure]: "<<"Moving to Point 1."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = -size_square/2;
    Command_Now.Reference_State.position_ref[1] = -size_square/2;
    Command_Now.Reference_State.position_ref[2] = height_square;
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    sleep(sleep_time);
        
    //第二个目标点，左上角
    cout<<"[sqaure]: "<<"Moving to Point 2."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = size_square/2;
    Command_Now.Reference_State.position_ref[1] = -size_square/2;
    Command_Now.Reference_State.position_ref[2] = height_square;
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    sleep(sleep_time);

    //第三个目标点，右上角
    cout<<"[sqaure]: "<<"Moving to Point 3."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = size_square/2;
    Command_Now.Reference_State.position_ref[1] = size_square/2;
    Command_Now.Reference_State.position_ref[2] = height_square;
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    sleep(sleep_time);

    //第四个目标点，右下角
    cout<<"[sqaure]: "<<"Moving to Point 4."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = -size_square/2;
    Command_Now.Reference_State.position_ref[1] = size_square/2;
    Command_Now.Reference_State.position_ref[2] = height_square;
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    sleep(sleep_time);

    //第五个目标点，回到起点
    cout<<"[sqaure]: "<<"Moving to Point 5."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0] = -size_square/2;
    Command_Now.Reference_State.position_ref[1] = -size_square/2;
    Command_Now.Reference_State.position_ref[2] = height_square;
    Command_Now.Reference_State.yaw_ref         = 0;

    move_pub.publish(Command_Now);

    sleep(sleep_time);

    //降落
    cout<<"[sqaure]: "<<"Landing."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                            = prometheus_msgs::ControlCommand::Land;
    Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
    move_pub.publish(Command_Now);

    sleep(1.0);

    return 0;
}
