#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <plan_manage/ego_replan_fsm.h>
#include <prometheus_msgs/UAVControlState.h>
using namespace ego_planner;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");
  EGOReplanFSM rebo_replan;
  rebo_replan.init(nh);
  ros::spin();
  return 0;
}

