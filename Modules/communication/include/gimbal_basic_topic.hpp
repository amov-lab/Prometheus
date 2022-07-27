#ifndef GimbalBasic_HPP
#define GimbalBasic_HPP

#include "communication.hpp"

#include "prometheus_msgs/GimbalState.h"
#include "prometheus_msgs/VisionDiff.h"
#include "prometheus_msgs/WindowPosition.h"
#include "prometheus_msgs/GimbalControl.h"

class GimbalBasic
{
public:
    GimbalBasic(ros::NodeHandle &nh);
    ~GimbalBasic();

    void stateCb(const prometheus_msgs::GimbalState::ConstPtr &msg);

    void trackCb(const prometheus_msgs::VisionDiff::ConstPtr &msg);

    void gimbalWindowPositionPub(struct WindowPosition window_position);

    void gimbalControlPub(struct GimbalControl gimbal_control);

protected:
    ros::Subscriber gimbal_state_sub_;
    ros::Subscriber vision_diff_sub_;
    ros::Publisher window_position_pub_;
    ros::Publisher gimbal_control_pub_;

    struct GimbalState gimbal_state_;
    struct VisionDiff vision_diff_;

    Communication* communication_ = NULL;
    std::string multicast_udp_ip;
};

#endif