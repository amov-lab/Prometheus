#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


#include "local_planning.h"


using namespace local_planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_planner_node");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  // 命名不规范
  PotentialFiledPlanner apl;
  apl.init(nh);

  ros::spin();

  return 0;
}