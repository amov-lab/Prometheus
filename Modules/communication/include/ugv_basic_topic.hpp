#ifndef UGV_BASIC_TOPIC_HPP
#define UGV_BASIC_TOPIC_HPP

#include <ros/ros.h>
#include "communication.hpp"
#include "prometheus_msgs/UGVCommand.h"
#include "prometheus_msgs/UGVState.h"
#include "prometheus_msgs/TextInfo.h"
#include "prometheus_msgs/CustomDataSegment.h"
#include "prometheus_msgs/BasicDataTypeAndValue.h"
#include "custom_data_segment.hpp"
#include "rviz_reduce_the_frequency.hpp"

using namespace std;

class UGVBasic
{
public:
    UGVBasic(ros::NodeHandle &nh,int id,Communication *communication);

    ~UGVBasic();

    void ugvCmdPub(struct UGVCommand ugv_command);

    void stateCb(const prometheus_msgs::UGVState::ConstPtr &msg);
    
    void textInfoCb(const prometheus_msgs::TextInfo::ConstPtr &msg);

    void customDataSegmentCb(const prometheus_msgs::CustomDataSegment::ConstPtr &msg);
    void customDataSegmentPub(struct CustomDataSegment_1 custom_data_segment);

    void setTimeStamp(uint time);

    uint getTimeStamp();

    struct UGVState getUGVState();

    void send(const ros::TimerEvent &time_event);

    void setGroundStationIP(std::string ip);
private:
    //
    ros::Publisher ugv_cmd_pub_;
    ros::Subscriber ugv_state_sub_;
    ros::Subscriber text_info_sub_;
    ros::Subscriber custom_data_segment_sub_;
    ros::Publisher custom_data_segment_pub_;

    Communication* communication_ = NULL;

    struct UGVState ugv_state_;
    struct TextInfo text_info_;
    struct CustomDataSegment_1 custom_data_segment_;

    bool custom_data_segment_ready = false;

    int robot_id;
    std::string udp_ip;
    std::string multicast_udp_ip;

    uint time_stamp_ = 0;

    // 只控制 ugv_state 的发送频率
    ros::Timer send_timer;
    int send_hz;
    // 下列变量仅在发送定时器中有效，为判断当前数据是否刷新
    bool ugv_state_ready = false;

    std::shared_ptr<ReduceTheFrequency> reduce_the_frequency_ = nullptr;
};

#endif