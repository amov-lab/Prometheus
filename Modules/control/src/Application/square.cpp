/***************************************************************************************************************************
 * square.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2018.8.17
 *
 * 说明: mavros正方形飞行示例程序
 *      1.
 *      2.
 *      3.
***************************************************************************************************************************/

#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <px4_command/ControlCommand.h>
#include <command_to_mavros.h>

using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
px4_command::ControlCommand Command_Now;
//---------------------------------------正方形参数---------------------------------------------
float size_square;                  //正方形边长
float height_square;                //飞行高度
float sleep_time;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "square");
    ros::NodeHandle nh("~");

    // 频率 [1hz]
    ros::Rate rate(1.0);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher move_pub = nh.advertise<px4_command::ControlCommand>("/px4_command/control_command", 10);

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

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主程序<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //takeoff
    i = 0;
    while (i < sleep_time)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = 0;
        Command_Now.Reference_State.position_ref[1] = 0;
        Command_Now.Reference_State.position_ref[2] = height_square;
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        rate.sleep();

        cout << "Point 1"<<endl;

        i++;

    }

    //依次发送4个目标点给position_control.cpp
    //第一个目标点，左下角
    i = 0;
    while (i < sleep_time)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = -size_square/2;
        Command_Now.Reference_State.position_ref[1] = -size_square/2;
        Command_Now.Reference_State.position_ref[2] = height_square;
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        rate.sleep();

        cout << "Point 1"<<endl;

        i++;

    }



    //第二个目标点，左上角
    i = 0;
    while (i < sleep_time)
    {

        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = size_square/2;
        Command_Now.Reference_State.position_ref[1] = -size_square/2;
        Command_Now.Reference_State.position_ref[2] = height_square;
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        rate.sleep();

        cout << "Point 2"<<endl;

        i++;

    }

    //第三个目标点，右上角
    i = 0;
    while (i < sleep_time)
    {
        Command_Now.header.stamp = ros::Time::now();
        
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = size_square/2;
        Command_Now.Reference_State.position_ref[1] = size_square/2;
        Command_Now.Reference_State.position_ref[2] = height_square;
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        rate.sleep();

        cout << "Point 3"<<endl;

        i++;

    }

    //第四个目标点，右下角
    i = 0;
    while (i < sleep_time)
    {

        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = -size_square/2;
        Command_Now.Reference_State.position_ref[1] = size_square/2;
        Command_Now.Reference_State.position_ref[2] = height_square;
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        rate.sleep();

        cout << "Point 4"<<endl;

        i++;

    }

    //第五个目标点，回到起点
    i = 0;
    while (i < sleep_time)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = -size_square/2;
        Command_Now.Reference_State.position_ref[1] = -size_square/2;
        Command_Now.Reference_State.position_ref[2] = height_square;
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        rate.sleep();

        cout << "Point 5"<<endl;

        i++;

    }

    //降落


    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode = command_to_mavros::Land;
    move_pub.publish(Command_Now);

    rate.sleep();

    cout << "Land"<<endl;





    return 0;
}
