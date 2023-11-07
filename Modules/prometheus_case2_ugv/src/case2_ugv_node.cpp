#include <ros/ros.h>

#include "case2_fsm_ugv.h"

using namespace prometheus_case2_ugv;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prometheus_case2_ugv");

  ros::NodeHandle nh("~");

  Case2FSM_UGV case2;
  case2.init(nh);

  ros::spin();

  return 0;
}

