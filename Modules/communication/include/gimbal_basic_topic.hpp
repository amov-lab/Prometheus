#ifndef GimbalBasic_HPP
#define GimbalBasic_HPP

#include <ros/ros.h>
#include "communication.hpp"

#include "prometheus_msgs/GimbalState.h"
#include "prometheus_msgs/VisionDiff.h"
#include "prometheus_msgs/WindowPosition.h"
#include "prometheus_msgs/GimbalControl.h"

class GimbalBasic
{
public:
    GimbalBasic(ros::NodeHandle &nh,Communication *communication);
    ~GimbalBasic();

    void stateCb(const prometheus_msgs::GimbalState::ConstPtr &msg);

    void trackCb(const prometheus_msgs::VisionDiff::ConstPtr &msg);

    void gimbalWindowPositionPub(struct WindowPosition window_position);

    void gimbalControlPub(struct GimbalControl gimbal_control);

    void send(const ros::TimerEvent &time_event);

protected:
    ros::Subscriber gimbal_state_sub_;
    ros::Subscriber vision_diff_sub_;
    ros::Publisher window_position_pub_;
    ros::Publisher gimbal_control_pub_;

    struct GimbalState gimbal_state_;
    struct VisionDiff vision_diff_;

    Communication* communication_ = NULL;
    std::string multicast_udp_ip;

    // 只控制 gimbal_state、vision_diff 的发送频率
    ros::Timer send_timer;
    int send_hz;
    // 下列变量仅在发送定时器中有效，为判断当前数据是否刷新
    bool gimbal_state_ready = false;
    bool vision_diff_ready = false;
};

#endif