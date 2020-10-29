#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "prometheus_msgs/PositionReference.h"

using namespace std;

// ros::Timer pub_odom_timer_;


ros::Publisher odom_pub;
ros::Publisher waypoint_pub;
nav_msgs::Odometry init_odom;
nav_msgs::Odometry odom_now;


bool is_run_odom;
int odom_mode = 2;
prometheus_msgs::PositionReference traj_now;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数定义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 【订阅】订阅轨迹，并赋值给odom，实现近似动态
void trajCallbck(const prometheus_msgs::PositionReference& msg){
    ROS_INFO("*******[uav sim]: recieve cmd *******");
    traj_now = msg; 
    odom_now.header.stamp = ros::Time::now();;
    odom_now.header.frame_id = "map";
    odom_now.pose.pose.position.x = traj_now.position_ref[0];
    odom_now.pose.pose.position.y = traj_now.position_ref[1];
    odom_now.pose.pose.position.z = traj_now.position_ref[2];

    odom_now.twist.twist.linear.x = traj_now.velocity_ref[0];
    odom_now.twist.twist.linear.y = traj_now.velocity_ref[1];
    odom_now.twist.twist.linear.z = traj_now.velocity_ref[2];
    is_run_odom = true;
}


// 【发布】处理里程计信息，根据模式选择是否发布
void omdpubCallback() {

    //we'll publish the odometry message over ROS
    if (is_run_odom==false){
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();;
        odom.header.frame_id = "map";

        //set the position and quaternion
        odom.pose.pose.position.x = init_odom.pose.pose.position.x;
        odom.pose.pose.position.y = init_odom.pose.pose.position.y;
        odom.pose.pose.position.z = init_odom.pose.pose.position.z;

        geometry_msgs::Quaternion odom_quat;
        odom_quat.x = 0;
        odom_quat.y = 0;
        odom_quat.z = 0;
        odom_quat.w = 0;
        odom.pose.pose.orientation = odom_quat;
        
        
        // 发布odom
        // printf("** publish init odom!**\n");
        odom_pub.publish(odom);

    }else if (is_run_odom==true && odom_mode==2) 
    {
        // should have some dynamics! 
        odom_pub.publish(odom_now);

    }
    
}

int main(int argc,char** argv)
{   
    // 1. initialization node
    ros::init(argc,argv,"uav_sim");
    ros::NodeHandle node_("~");
    // 2. 发布odom和点云地图
    odom_pub = node_.advertise<nav_msgs::Odometry>("/prometheus/drone_odom", 50);



    node_.param("test/init_odom_x", init_odom.pose.pose.position.x, 0.0);
    node_.param("test/init_odom_y", init_odom.pose.pose.position.y, 0.0);
    node_.param("test/init_odom_z", init_odom.pose.pose.position.z, 1.0);

    is_run_odom = false;

    // 3. set the trigger frequece for different events. 

    ros::Subscriber traj_sub = node_.subscribe("/prometheus/fast_planner/position_cmd", 50, trajCallbck);
    // ros::Timer pub_odom_timer_ = node_.createTimer(ros::Duration(0.02), omdpubCallback);

    ROS_INFO(" the simulator initialization successful!");

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        omdpubCallback();
        ros::spinOnce();
        loop_rate.sleep();
    }

}



