#include <ros/ros.h>
#include "drone_detector/drone_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_detect");
  ros::NodeHandle nh("~");

  detect::DroneDetector drone_detector(nh);
  drone_detector.test();

  ros::spin();
  return 0;
}
