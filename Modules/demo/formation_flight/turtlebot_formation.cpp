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
#include <prometheus_msgs/PositionReference.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include "message_utils.h"

using namespace std;

# define NODE_NAME "turtlebot_formation"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
nav_msgs::Odometry tb3_1_odom;
nav_msgs::Odometry tb3_2_odom;
nav_msgs::Odometry tb3_3_odom;
nav_msgs::Odometry tb3_4_odom;

geometry_msgs::Twist tb3_1_cmd;
geometry_msgs::Twist tb3_2_cmd;
geometry_msgs::Twist tb3_3_cmd;
geometry_msgs::Twist tb3_4_cmd;

void tb3_1_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    tb3_1_odom = *msg;
}

void tb3_2_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    tb3_2_odom = *msg;
}

void tb3_3_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    tb3_3_odom = *msg;
}

void tb3_4_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    tb3_4_odom = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_formation");
    ros::NodeHandle nh("~");

    ros::Subscriber tb3_1_odom_sub = nh.subscribe<nav_msgs::Odometry>("/tb3_1/odom", 10, tb3_1_odom_cb);
    ros::Subscriber tb3_2_odom_sub = nh.subscribe<nav_msgs::Odometry>("/tb3_2/odom", 10, tb3_2_odom_cb);
    ros::Subscriber tb3_3_odom_sub = nh.subscribe<nav_msgs::Odometry>("/tb3_3/odom", 10, tb3_3_odom_cb);
    ros::Subscriber tb3_4_odom_sub = nh.subscribe<nav_msgs::Odometry>("/tb3_4/odom", 10, tb3_4_odom_cb);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    ros::Publisher tb3_1_cmd_pub = nh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 10);
    ros::Publisher tb3_2_cmd_pub = nh.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel", 10);
    ros::Publisher tb3_3_cmd_pub = nh.advertise<geometry_msgs::Twist>("/tb3_3/cmd_vel", 10);
    ros::Publisher tb3_4_cmd_pub = nh.advertise<geometry_msgs::Twist>("/tb3_4/cmd_vel", 10);

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

    // Waiting for input
    int start_flag = 0;

    ros::Rate rate(50.0);

    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>turtlebot_formation Mission<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to move the turtlebot."<<endl;
        cin >> start_flag;
    }

    float time_trajectory = 0.0;

    ros::Duration(2.0).sleep();

    float x_sp,y_sp;
    while (ros::ok())
    {
        // turtlebot3命令发布
        tb3_1_cmd.linear.x = 0.4;
        tb3_1_cmd.linear.y = 0.0;
        tb3_1_cmd.linear.z = 0.0;

        tb3_1_cmd.angular.x = 0.0;
        tb3_1_cmd.angular.y = 0.0;
        tb3_1_cmd.angular.z = 0.2;

        tb3_2_cmd.linear.x = 0.2;
        tb3_2_cmd.linear.y = 0.0;
        tb3_2_cmd.linear.z = 0.0;

        tb3_2_cmd.angular.x = 0.0;
        tb3_2_cmd.angular.y = 0.0;
        tb3_2_cmd.angular.z = 0.2;

        tb3_3_cmd.linear.x = 0.2;
        tb3_3_cmd.linear.y = 0.0;
        tb3_3_cmd.linear.z = 0.0;

        tb3_3_cmd.angular.x = 0.0;
        tb3_3_cmd.angular.y = 0.0;
        tb3_3_cmd.angular.z = 0.2;

        tb3_4_cmd.linear.x = 0.4;
        tb3_4_cmd.linear.y = 0.0;
        tb3_4_cmd.linear.z = 0.0;

        tb3_4_cmd.angular.x = 0.0;
        tb3_4_cmd.angular.y = 0.0;
        tb3_4_cmd.angular.z = 0.2;

        tb3_1_cmd_pub.publish(tb3_1_cmd);
        tb3_2_cmd_pub.publish(tb3_2_cmd);
        tb3_3_cmd_pub.publish(tb3_3_cmd);
        tb3_4_cmd_pub.publish(tb3_4_cmd);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;

}
