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
    ros::init(argc, argv, "pub_goal");
    ros::NodeHandle nh("~");

    //【发布】目标点
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 10);

    float x,y,z;

    geometry_msgs::PoseStamped goal;
    int flag;
    cout << "Please choose 2D or 3D (0 for 2D, 1 for 3D):"<<endl;
    cin >> flag;  
  
    while(ros::ok())
    {

        // Waiting for input
        cout << "Please input the goal position:"<<endl;
        cout << "goal - x [m] : "<< endl;
        cin >> x;
        cout << "goal -  y [m] : "<< endl;
        cin >> y;
        if(flag == 1)
        {
            cout << "goal -  z [m] : "<< endl;
            cin >> z;
        }else if(flag == 0)
        {
            z = 1.0;
        }


        goal.header.stamp =ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1.0;


        goal_pub.publish(goal);

        sleep(1.0);
    }

   
    return 0;

}
