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


#define FIVE_STAR_SIZE 2.0
#define TRIANGLE_SIZE 2.0
#define T_SIZE 0.8
# define NODE_NAME "turtlebot_uav_formation"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
nav_msgs::Odometry tb3_1_odom;
nav_msgs::Odometry tb3_2_odom;
nav_msgs::Odometry tb3_3_odom;
nav_msgs::Odometry tb3_4_odom;

geometry_msgs::Twist tb3_1_cmd;
geometry_msgs::Twist tb3_2_cmd;
geometry_msgs::Twist tb3_3_cmd;
geometry_msgs::Twist tb3_4_cmd;


geometry_msgs::Point leader;
geometry_msgs::Point relative_pos_to_leader1;
geometry_msgs::Point relative_pos_to_leader2;
geometry_msgs::Point relative_pos_to_leader3;
geometry_msgs::Point relative_pos_to_leader4;
geometry_msgs::Point relative_pos_to_leader5;

geometry_msgs::PoseStamped goal;
int flag_get_goal;

ros::Publisher leader_pub;
ros::Publisher formation_pub1;
ros::Publisher formation_pub2;
ros::Publisher formation_pub3;
ros::Publisher formation_pub4;
ros::Publisher formation_pub5;

float uav_number;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void five_star();
void triangle();
void square_shape();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
    flag_get_goal = 1;
    cout << "Get a new goal from rviz!"<<endl;

    //高度打死
    leader.x = goal.pose.position.x;
    leader.y = goal.pose.position.y;
    leader.z = 1.0;
}

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
    ros::init(argc, argv, "turtlebot_uav_formation");
    ros::NodeHandle nh("~");

    nh.param<float>("uav_number", uav_number, 5);
    
    //【订阅】目标点
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/formation/goal", 10, goal_cb);


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


    //【发布】阵型
    leader_pub     = nh.advertise<geometry_msgs::Point>("/prometheus/formation/leader_pos", 10);
    formation_pub1 = nh.advertise<geometry_msgs::Point>("/prometheus/formation/uav1", 10);
    formation_pub2 = nh.advertise<geometry_msgs::Point>("/prometheus/formation/uav2", 10);
    formation_pub3 = nh.advertise<geometry_msgs::Point>("/prometheus/formation/uav3", 10);
    formation_pub4 = nh.advertise<geometry_msgs::Point>("/prometheus/formation/uav4", 10);
    formation_pub5 = nh.advertise<geometry_msgs::Point>("/prometheus/formation/uav5", 10);

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
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to takeoff all the UAVs."<<endl;
        cin >> start_flag;
        leader_pub.publish(leader);
    }

    float time_trajectory = 0.0;

    ros::Duration(2.0).sleep();

    float x_sp,y_sp;
    while (ros::ok())
    {
        float linear_vel = 0.5;
        float circle_radius = 2.0;
        float omega;
        omega =  fabs(linear_vel / circle_radius);

        Eigen::Vector3f circle_center;
        circle_center[0] = 0.0;
        circle_center[1] = 0.0;
        float angle = time_trajectory * omega;
        float cos_angle = cos(angle);
        float sin_angle = sin(angle);
        time_trajectory = time_trajectory + 0.02;

        leader.x = circle_radius * cos_angle + circle_center[0];
        leader.y = circle_radius * sin_angle + circle_center[1];
        leader.z = 1.0;
        //发布
        leader_pub.publish(leader);
        //triangle();
        square_shape();

        // turtlebot3命令发布
        float kp = 1;
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

        // pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Switch to five_star formation.");
  
        cout << "Leader [X Y Z] : " << leader.x << " [ m ] "<< leader.y <<" [ m ] "<< leader.z <<" [ m ] "<<endl;
     
        rate.sleep();
        ros::spinOnce();
    }

    return 0;

}



void triangle()
{   
    relative_pos_to_leader1.x = TRIANGLE_SIZE;
    relative_pos_to_leader1.y = TRIANGLE_SIZE;
    relative_pos_to_leader1.z = 0.0;
    formation_pub1.publish(relative_pos_to_leader1);

    relative_pos_to_leader2.x = 0.0;
    relative_pos_to_leader2.y = 2 * TRIANGLE_SIZE;
    relative_pos_to_leader2.z = 0.0;
    formation_pub2.publish(relative_pos_to_leader2);

    relative_pos_to_leader3.x = 0.0;
    relative_pos_to_leader3.y = - 2 * TRIANGLE_SIZE;
    relative_pos_to_leader3.z = 0.0;
    formation_pub3.publish(relative_pos_to_leader3);

    relative_pos_to_leader4.x = TRIANGLE_SIZE;
    relative_pos_to_leader4.y = -TRIANGLE_SIZE;
    relative_pos_to_leader4.z = 0.0;
    formation_pub4.publish(relative_pos_to_leader4);

    relative_pos_to_leader5.x = 2 * TRIANGLE_SIZE;
    relative_pos_to_leader5.y = 0.0;
    relative_pos_to_leader5.z = 0.0;
    formation_pub5.publish(relative_pos_to_leader5);
}
void square_shape()
{
    relative_pos_to_leader1.x =   T_SIZE;
    relative_pos_to_leader1.y =   T_SIZE;
    relative_pos_to_leader1.z = 0.0;
    formation_pub1.publish(relative_pos_to_leader1);

    relative_pos_to_leader2.x = - T_SIZE;
    relative_pos_to_leader2.y =   T_SIZE;
    relative_pos_to_leader2.z = 0.0;
    formation_pub2.publish(relative_pos_to_leader2);

    relative_pos_to_leader3.x = - T_SIZE;
    relative_pos_to_leader3.y = - T_SIZE;
    relative_pos_to_leader3.z = 0.0;
    formation_pub3.publish(relative_pos_to_leader3);

    relative_pos_to_leader4.x = T_SIZE;
    relative_pos_to_leader4.y = - T_SIZE;
    relative_pos_to_leader4.z = 0.0;
    formation_pub4.publish(relative_pos_to_leader4);

}