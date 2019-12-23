/***************************************************************************************************************************
 * payload_drop.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2019.06.07
 *
 * Introduction: payload_drop.cpp
 * 1.Takeoff
 * 2.Fly to the target position
 * 3.Decrease the altitude
 * 4.Drop
 * 5.Increase the altitude
 * 6.Back to the takeoff point
 * 7.Land
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
#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>

using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::ControlCommand Command_Now;
//---------------------------------------正方形参数---------------------------------------------
float fly_height;
float target_x,target_y;
float drop_height;
float drop_flag;
float limit_time;
float min_distance;
float distance_to_target;

int time_sec = 0;
int comid = 0;
int switch_flag = 0;

Eigen::Vector3f drone_pos;
Eigen::Vector3f point1;
Eigen::Vector3f point2;
Eigen::Vector3f point3;

float cal_distance(Eigen::Vector3f a,Eigen::Vector3f b);
void printf_result(Eigen::Vector3f target_point);

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    drone_pos  = Eigen::Vector3f(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "payload_drop");
    ros::NodeHandle nh("~");

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    nh.param<float>("fly_height", fly_height, 1.0);
    nh.param<float>("target_x", target_x, 0.0);
    nh.param<float>("target_y", target_y, 0.0);
    nh.param<float>("drop_height", drop_height, 1.5);
    nh.param<float>("limit_time", limit_time, 10.0);
    nh.param<float>("min_distance", min_distance, 0.2);

    // 频率 [1hz]
    ros::Rate rate(1.0);

    //Subscribe the drone position
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus_msgs/control_command", 10);

    // Drop cmd send to mavros
    ros::Publisher drop_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    mavros_msgs::OverrideRCIn drop_cmd;

    drop_cmd.channels[0] = 0;
    drop_cmd.channels[1] = 0;
    drop_cmd.channels[2] = 0;
    drop_cmd.channels[3] = 0;
    drop_cmd.channels[4] = 0;
    drop_cmd.channels[5] = 0;
    drop_cmd.channels[6] = 0;
    drop_cmd.channels[7] = 1600;

    drop_flag = 0;

    int check_flag;
    // Check the parameter
    cout << "fly_height: "<<fly_height<<"[m]"<<endl;
    cout << "target_x: "<<target_x<<"[m]"<<endl;
    cout << "target_y: "<<target_y<<"[m]"<<endl;
    cout << "drop_height: "<<drop_height<<"[m]"<<endl;
    cout << "limit_time: "<<limit_time<<"[s]"<<endl;
    cout << "min_distance: "<<min_distance<<"[m]"<<endl;

    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    //Read the takeoff point
    int i =0;
    for(i=0;i<5;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    point1[0] = drone_pos[0];
    point1[1] = drone_pos[1];
    point1[2] = fly_height;

    point2[0] = target_x;
    point2[1] = target_y;
    point2[2] = fly_height;

    point3[0] = target_x;
    point3[1] = target_y;
    point3[2] = drop_height;


    //Takeoff - fly to point 1
    while (switch_flag == 0)
    {
        Command_Now.Mode = Command_Now.Move_ENU;         //Move模式
        Command_Now.Reference_State.Sub_mode  = Command_Now.Reference_State.XYZ_POS;               //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point1[0];
        Command_Now.Reference_State.position_ref[1] = point1[1];
        Command_Now.Reference_State.position_ref[2] = point1[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point1);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        printf_result(point1);

        cout << "Takeoff: flying to Point 1"<<endl;

        rate.sleep();
        ros::spinOnce();

    }

    //wait 1s more
    sleep(1.0);
    //reset the flag
    switch_flag = 0;
    time_sec = 0;

    //Fly to target - to point 2
    while (switch_flag == 0)
    {
        Command_Now.Mode = Command_Now.Move_ENU;         //Move模式
        Command_Now.Reference_State.Sub_mode  = Command_Now.Reference_State.XYZ_POS;               //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point2[0];
        Command_Now.Reference_State.position_ref[1] = point2[1];
        Command_Now.Reference_State.position_ref[2] = point2[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point2);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        printf_result(point2);

        cout << "Fly to target - Flying to Point 2"<<endl;

        rate.sleep();
        ros::spinOnce();

    }

    //wait 1s more
    sleep(1.0);
    //reset the flag
    switch_flag = 0;
    time_sec = 0;


    //Point 3 get target and decrease the height

    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = Command_Now.Move_ENU;         //Move模式
        Command_Now.Reference_State.Sub_mode  = Command_Now.Reference_State.XYZ_POS;               //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point3[0];
        Command_Now.Reference_State.position_ref[1] = point3[1];
        Command_Now.Reference_State.position_ref[2] = point3[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point3);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        printf_result(point3);

        cout << "Get target and decrease the height- flying to Point 3"<<endl;

        rate.sleep();
        ros::spinOnce();

    }

    //wait 1s more
    sleep(1.0);
    //reset the flag
    switch_flag = 0;
    time_sec = 0;

    //Drop
    while (time_sec < 3)
    {
        drop_pub.publish(drop_cmd);

        cout << "Droping......"<<endl;

        time_sec++;
        rate.sleep();
        ros::spinOnce();
    }

    //wait 1s more
    sleep(1.0);
    //reset the flag
    switch_flag = 0;
    time_sec = 0;

    //Finsh drop and increase the altitude,fly to point2
    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = Command_Now.Move_ENU;         //Move模式
        Command_Now.Reference_State.Sub_mode  = Command_Now.Reference_State.XYZ_POS;               //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point2[0];
        Command_Now.Reference_State.position_ref[1] = point2[1];
        Command_Now.Reference_State.position_ref[2] = point2[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point2);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        printf_result(point2);

        cout << "Finsh drop and increase the altitude...fly to point2"<<endl;

        rate.sleep();
        ros::spinOnce();

    }

    //wait 1s more
    sleep(1.0);
    //reset the flag
    switch_flag = 0;
    time_sec = 0;


    //Go back to the takeoff point - point1
    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = Command_Now.Move_ENU;         //Move模式
        Command_Now.Reference_State.Sub_mode  = Command_Now.Reference_State.XYZ_POS;               //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point1[0];
        Command_Now.Reference_State.position_ref[1] = point1[1];
        Command_Now.Reference_State.position_ref[2] = point1[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point1);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        printf_result(point1);

        cout << "Go back to the takeoff point...flying to point 1"<<endl;

        rate.sleep();
        ros::spinOnce();

    }

    //wait 1s more
    sleep(1.0);
    time_sec = 0;


    // Land
    while (time_sec < 5)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = Command_Now.Land;
        move_pub.publish(Command_Now);

        cout << "Finsh the payload drop mission, Landing..."<<endl;

        time_sec++;
        rate.sleep();
        ros::spinOnce();
    }


    return 0;
}

float cal_distance(Eigen::Vector3f a,Eigen::Vector3f b)
{
    float distance;
    distance = sqrt(  (a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1]) + (a[2] - b[2])*(a[2] - b[2]) );
    return distance;
}

void printf_result(Eigen::Vector3f target_point)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Payload Drop Mission<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

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

    cout << "Drone_Pos  [X Y Z] : " << drone_pos[0] << " [ m ] "<< drone_pos[1]<<" [ m ] "<<drone_pos[2]<<" [ m ] "<<endl;
    cout << "Target_Pos [X Y Z] : " << target_point[0] << " [ m ] "<< target_point[1]<<" [ m ] "<<target_point[2]<<" [ m ] "<<endl;
    cout << "Distance_to_target : " << distance_to_target <<" [ m ] "<<endl;
    cout << "Flying_Time        : " << time_sec <<" [ s ] "<<endl;
    cout << "Drop_Flag          : " << drop_flag <<endl;

}
