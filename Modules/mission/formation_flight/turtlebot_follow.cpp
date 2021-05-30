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

prometheus_msgs::DroneState uav1_state;
prometheus_msgs::DroneState uav2_state;
prometheus_msgs::DroneState uav3_state;
prometheus_msgs::DroneState uav4_state;

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

void uav1_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    uav1_state = *msg;
}
void uav2_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    uav2_state = *msg;
}
void uav3_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    uav3_state = *msg;
}
void uav4_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    uav4_state = *msg;
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

    ros::Subscriber uav1_sub = nh.subscribe<prometheus_msgs::DroneState>("/uav1/prometheus/drone_state", 1, uav1_cb);
    ros::Subscriber uav2_sub = nh.subscribe<prometheus_msgs::DroneState>("/uav2/prometheus/drone_state", 1, uav2_cb);
    ros::Subscriber uav3_sub = nh.subscribe<prometheus_msgs::DroneState>("/uav3/prometheus/drone_state", 1, uav3_cb);
    ros::Subscriber uav4_sub = nh.subscribe<prometheus_msgs::DroneState>("/uav4/prometheus/drone_state", 1, uav4_cb);

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

    float kp = 0.5;

    float x_sp,y_sp;
    while (ros::ok())
    {
        // turtlebot3命令发布
        tb3_1_cmd.linear.x = kp * (uav1_state.position[0] - tb3_1_odom.pose.pose.position.x);
        tb3_1_cmd.linear.y = kp * (uav1_state.position[1] - tb3_1_odom.pose.pose.position.y);
        tb3_1_cmd.linear.z = 0.0;

        tb3_1_cmd.angular.x = 0.0;
        tb3_1_cmd.angular.y = 0.0;
        tb3_1_cmd.angular.z = 0.0;

        tb3_2_cmd.linear.x = kp * (uav2_state.position[0] - tb3_2_odom.pose.pose.position.x);
        tb3_2_cmd.linear.y = kp * (uav2_state.position[1] - tb3_2_odom.pose.pose.position.x);
        tb3_2_cmd.linear.z = 0.0;

        tb3_2_cmd.angular.x = 0.0;
        tb3_2_cmd.angular.y = 0.0;
        tb3_2_cmd.angular.z = 0.0;

        tb3_3_cmd.linear.x = kp * (uav3_state.position[0] - tb3_3_odom.pose.pose.position.x);
        tb3_3_cmd.linear.y = kp * (uav3_state.position[1] - tb3_3_odom.pose.pose.position.x);
        tb3_3_cmd.linear.z = 0.0;

        tb3_3_cmd.angular.x = 0.0;
        tb3_3_cmd.angular.y = 0.0;
        tb3_3_cmd.angular.z = 0.0;

        tb3_4_cmd.linear.x = kp * (uav4_state.position[0] - tb3_4_odom.pose.pose.position.x);
        tb3_4_cmd.linear.y = kp * (uav4_state.position[1] - tb3_4_odom.pose.pose.position.x);
        tb3_4_cmd.linear.z = 0.0;

        tb3_4_cmd.angular.x = 0.0;
        tb3_4_cmd.angular.y = 0.0;
        tb3_4_cmd.angular.z = 0.0;

        tb3_1_cmd_pub.publish(tb3_1_cmd);
        tb3_2_cmd_pub.publish(tb3_2_cmd);
        tb3_3_cmd_pub.publish(tb3_3_cmd);
        tb3_4_cmd_pub.publish(tb3_4_cmd);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;

}
