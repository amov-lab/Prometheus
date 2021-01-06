#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


#include "local_planning.h"


using namespace Local_Planning;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle nh("~");

  Local_Planner local_planning;
  local_planning.init(nh);

  ros::spin();

  return 0;
}