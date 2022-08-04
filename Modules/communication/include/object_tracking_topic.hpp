#ifndef OBJECT_TRACKING_TOPIC_HPP
#define OBJECT_TRACKING_TOPIC_HPP

#include <ros/ros.h>
#include "communication.hpp"
#include "prometheus_msgs/MultiDetectionInfoSub.h"
#include "prometheus_msgs/DetectionInfoSub.h"

class ObjectTracking
{
public:
    ObjectTracking(ros::NodeHandle &nh,Communication *communication);
    ~ObjectTracking();

    void multiDetectionInfoCb(const prometheus_msgs::MultiDetectionInfoSub::ConstPtr &msg);


private:
    Communication* communication_ = NULL;

    std::string multicast_udp_ip;

    ros::Subscriber multi_detection_info_sub_;
};

#endif
