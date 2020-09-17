#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


#include "global_planning.h"



using namespace global_planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner_node");

  ros::NodeHandle nh("~");

  GlobalPlanner gpl;
  gpl.init(nh);

  ros::spin();

  return 0;
}

