#include <ros/ros.h>

#include "global_planner.h"
#include <signal.h>

using namespace GlobalPlannerNS;

void mySigintHandler(int sig)
{
  ROS_INFO("[global_planner_node] exit...");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh("~");

  signal(SIGINT, mySigintHandler);
  ros::Duration(1.0).sleep();

  GlobalPlanner global_planner(nh);

  ros::spin();

  return 0;
}

