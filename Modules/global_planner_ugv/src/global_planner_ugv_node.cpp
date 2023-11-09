#include <ros/ros.h>

#include "global_planner_ugv.h"

using namespace global_planner_ugv;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner_ugv");

  ros::NodeHandle nh("~");

  GlobalPlannerUGV ugv_global_planner;
  ugv_global_planner.init(nh);

  ros::spin();

  return 0;
}

