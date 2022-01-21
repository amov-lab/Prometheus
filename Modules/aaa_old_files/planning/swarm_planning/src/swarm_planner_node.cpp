#include <ros/ros.h>

#include "swarm_planner.h"

using namespace Swarm_Planning;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "swarm_planner");

  ros::NodeHandle nh("~");

  Swarm_Planner swarm_planner;
  swarm_planner.init(nh);

  ros::spin();

  return 0;
}

