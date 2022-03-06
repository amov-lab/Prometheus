#include <ros/ros.h>
#include <signal.h>

#include "local_planner.h"

using namespace LocalPlannerNS;

void mySigintHandler(int sig)
{
  ROS_INFO("[local_planner_node] exit...");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle nh("~");

  signal(SIGINT, mySigintHandler);
  ros::Duration(1.0).sleep();

  LocalPlanner local_planner(nh);

  ros::spin();

  return 0;
}