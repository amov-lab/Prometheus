#ifndef UGV_BASIC_TOPIC_HPP
#define UGV_BASIC_TOPIC_HPP

#include <ros/ros.h>
#include "communication.hpp"
#include "prometheus_msgs/UGVCommand.h"
#include "prometheus_msgs/UGVState.h"

using namespace std;

class UGVBasic
{
public:
    UGVBasic(ros::NodeHandle &nh,int id,Communication *communication);

    ~UGVBasic();

    void ugvCmdPub(struct UGVCommand ugv_command);

    void stateCb(const prometheus_msgs::UGVState::ConstPtr &msg);

    void setTimeStamp(uint time);

    uint getTimeStamp();

    struct UGVState getUGVState();
private:
    //
    ros::Publisher ugv_cmd_pub_;
    ros::Subscriber ugv_state_sub_;

    Communication* communication_ = NULL;

    struct UGVState ugv_state_;

    int robot_id;
    std::string udp_ip;
    std::string multicast_udp_ip;

    uint time_stamp_ = 0;
};

#endif