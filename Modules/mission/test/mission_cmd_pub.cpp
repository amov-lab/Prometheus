//ros头文件
#include <ros/ros.h>
#include <iostream>
#include <mission_utils.h>
#include "message_utils.h"

//topic 头文件
#include <geometry_msgs/PoseStamped.h>

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_cmd_pub");
    ros::NodeHandle nh("~");

    //【发布】目标点
    ros::Publisher mission_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/prometheus/mission/cmd", 10);

    float x,y,z;

    geometry_msgs::PoseStamped mission_cmd;
  
    while(ros::ok())
    {

        // Waiting for input
        cout << "Please input the mission_cmd:"<<endl;
        cout << "mission_cmd - x [m] : "<< endl;
        cin >> x;
        cout << "mission_cmd -  y [m] : "<< endl;
        cin >> y;

        mission_cmd.header.stamp =ros::Time::now();
        mission_cmd.header.frame_id = "map";
        mission_cmd.pose.position.x = x;
        mission_cmd.pose.position.y = y;
        mission_cmd.pose.position.z = 0;
        mission_cmd.pose.orientation.x = 0.0;
        mission_cmd.pose.orientation.y = 0.0;
        mission_cmd.pose.orientation.z = 0.0;
        mission_cmd.pose.orientation.w = 1.0;


        mission_cmd_pub.publish(mission_cmd);

        sleep(1.0);
    }

   
    return 0;

}
