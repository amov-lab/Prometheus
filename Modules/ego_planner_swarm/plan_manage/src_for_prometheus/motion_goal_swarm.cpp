#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <unistd.h>
#include <sstream>
#include "printf_utils.h"

double target[3][20];
ros::Publisher motion_goal_pub[10];
ros::Subscriber motion_goal_sub;
void pub_motion_goal(const geometry_msgs::PoseStamped::ConstPtr &msg, ros::Publisher motion_goal_pub , double offset_y){
  geometry_msgs::PoseStamped swarm_motion_goal;
  swarm_motion_goal.header.frame_id = msg->header.frame_id;
  swarm_motion_goal.header.stamp = ros::Time::now();
  swarm_motion_goal.pose.position.x = msg->pose.position.x;
  swarm_motion_goal.pose.position.y = msg->pose.position.y + offset_y ;
  swarm_motion_goal.pose.position.z = msg->pose.position.z;
  swarm_motion_goal.pose.orientation.w = msg->pose.orientation.w;
  swarm_motion_goal.pose.orientation.x = msg->pose.orientation.x;
  swarm_motion_goal.pose.orientation.y = msg->pose.orientation.y;
  swarm_motion_goal.pose.orientation.z = msg->pose.orientation.z;
  motion_goal_pub.publish(swarm_motion_goal);
}
void goal_sub_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
  geometry_msgs::PoseStamped swarm_motion_goal1,swarm_motion_goal2 , swarm_motion_goal3; 
  pub_motion_goal(msg,motion_goal_pub[1],0);
  pub_motion_goal(msg,motion_goal_pub[2],1);
  pub_motion_goal(msg,motion_goal_pub[3],-1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planning_pub");
    ros::NodeHandle nh("~");
    // 集群数量
    int num_of_swarm = 3;
    string uav_name;
    motion_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/motion_planning/goal", 10 , goal_sub_callback);
    for (int i = 1; i < num_of_swarm + 1; i++)
    {
       // 【发布】目标点至EGO-planner-swarm
      motion_goal_pub[i] = nh.advertise<geometry_msgs::PoseStamped>( "/uav" + std::to_string(i) + "/prometheus/motion_planning/goal",10);
    } 
    ros::Time last_t = ros::Time::now();
    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}