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
    ros::init(argc, argv, "pub_goal_swarm");
    ros::NodeHandle nh("~");

    //【发布】目标点
    ros::Publisher goal_pub1 = nh.advertise<geometry_msgs::PoseStamped>("/uav1/prometheus/planning/goal", 10);
    ros::Publisher goal_pub2 = nh.advertise<geometry_msgs::PoseStamped>("/uav2/prometheus/planning/goal", 10);
    ros::Publisher goal_pub3 = nh.advertise<geometry_msgs::PoseStamped>("/uav3/prometheus/planning/goal", 10);

    float x,y1,y2,y3,z;

    geometry_msgs::PoseStamped goal1;
    geometry_msgs::PoseStamped goal2;
    geometry_msgs::PoseStamped goal3;
    int flag;
    cout << "Please choose 2D or 3D (0 for 2D, 1 for 3D):"<<endl;
    cin >> flag;  
  
    while(ros::ok())
    {

        // Waiting for input
        cout << "Please input the goal position:"<<endl;
        cout << "goal - x [m] : "<< endl;
        cin >> x;
        cout << "goal -  y_uav1 [m] : "<< endl;
        cin >> y1;
        cout << "goal -  y_uav2 [m] : "<< endl;
        cin >> y2;
        cout << "goal -  y_uav2 [m] : "<< endl;
        cin >> y3;
        if(flag == 1)
        {
            cout << "goal -  z [m] : "<< endl;
            cin >> z;
        }else if(flag == 0)
        {
            z = 1.0;
        }


        goal1.header.stamp =ros::Time::now();
        goal1.header.frame_id = "map";
        goal1.pose.position.x = x;
        goal1.pose.position.y = y1;
        goal1.pose.position.z = z;
        goal1.pose.orientation.x = 0.0;
        goal1.pose.orientation.y = 0.0;
        goal1.pose.orientation.z = 0.0;
        goal1.pose.orientation.w = 1.0;
        goal2 = goal1;

        goal3 = goal1;

        goal2.pose.position.y = y2;
        goal3.pose.position.y = y3;


        goal_pub1.publish(goal1);
        goal_pub2.publish(goal2);
        goal_pub3.publish(goal3);

        sleep(1.0);
    }

   
    return 0;

}
