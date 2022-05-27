#include <ros/ros.h>
#include <signal.h>

#include "rapid_quad_trajectory.h"

void mySigintHandler(int sig)
{
  ROS_INFO("[rapid_quad_trajectory_node] exit...");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rapid_quad_trajectory_node");
  ros::NodeHandle nh("~");

  signal(SIGINT, mySigintHandler);
  ros::Duration(1.0).sleep();

  RapidQuadTrajectory RapidQuadTrajectory_(nh);

  ros::spin();

  return 0;
}