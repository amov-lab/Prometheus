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

float setpoint_length;                  //正方形边长
float T_constant;                    //飞行高度
float total_times;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "setpoint_track");
    ros::NodeHandle nh("~");

    // 频率 [1hz]
    ros::Rate rate(1.0);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher move_pub = nh.advertise<px4_command::ControlCommand>("/px4_command/control_command", 10);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    nh.param<float>("setpoint_length", setpoint_length, 0.5);
    nh.param<float>("T_constant", T_constant, 10.0);
    nh.param<float>("total_times", total_times, 40.0);



    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "setpoint_length: "<<setpoint_length<<"[m]"<<endl;
    cout << "T_constant: "<<T_constant<<"[m]"<<endl;
    cout << "total_times: "<<total_times<<"[]"<<endl;

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
    while (i < total_times)
    {

        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        
        Command_Now.Reference_State.position_ref[1] = 0;
        Command_Now.Reference_State.position_ref[2] = 1.0;
        Command_Now.Reference_State.velocity_ref[0] = 0;
        Command_Now.Reference_State.velocity_ref[1] = 0;
        Command_Now.Reference_State.velocity_ref[2] = 0;
        Command_Now.Reference_State.acceleration_ref[0] = 0;
        Command_Now.Reference_State.acceleration_ref[1] = 0;
        Command_Now.Reference_State.acceleration_ref[2] = 0;
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        if( i%2 == 0)
        {
            Command_Now.Reference_State.position_ref[0] = setpoint_length;
        }else 
        {
            Command_Now.Reference_State.position_ref[0] = - setpoint_length;
        }
        
        move_pub.publish(Command_Now);

        cout << "Point:"<< i <<endl;

        i++;

        sleep(T_constant);
    }

    //降落


    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode = command_to_mavros::Land;
    move_pub.publish(Command_Now);

    rate.sleep();

    cout << "Land"<<endl;
    return 0;
}
