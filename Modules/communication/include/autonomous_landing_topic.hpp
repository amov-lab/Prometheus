#ifndef AUTONOMOUS_LANDING_TOPIC_HPP
#define AUTONOMOUS_LANDING_TOPIC_HPP

#include <ros/ros.h>
#include "communication.hpp"

#include "gimbal_basic_topic.hpp"

#include "std_srvs/SetBool.h"
#include "mavros_msgs/ParamSet.h"


class AutonomousLanding
{
public:
    AutonomousLanding(ros::NodeHandle &nh,Communication *communication);

    ~AutonomousLanding();


    void gimbalSearchServer(bool is);

    void gimbalRecordVideoServer(bool is);

    void gimbalTrackModeServer(bool is);

    void gimbalParamSetServer(struct GimbalParamSet param_set);

private:

    ros::Publisher ugv_state_pub_;

    ros::ServiceClient gimbal_search_client_;
    ros::ServiceClient gimbal_record_video_client_;
    ros::ServiceClient gimbal_track_mode_client_;
    ros::ServiceClient param_set_client_;

    struct GimbalState gimbal_state_;
    struct VisionDiff vision_diff_;

    int robot_id;

    Communication* communication_ = NULL;
    std::string multicast_udp_ip;
};

#endif