#include "object_tracking_topic.hpp"

ObjectTracking::ObjectTracking(ros::NodeHandle &nh,Communication *communication)
{
    nh.param<std::string>("multicast_udp_ip", this->multicast_udp_ip, "224.0.0.88");
    this->communication_ = communication; 
    this->multi_detection_info_sub_ = nh.subscribe("/deepsort_ros/object_detection_result", 10, &ObjectTracking::multiDetectionInfoCb, this);
}
ObjectTracking::~ObjectTracking()
{
    delete this->communication_;
}

void ObjectTracking::multiDetectionInfoCb(const prometheus_msgs::MultiDetectionInfoSub::ConstPtr &msg)
{
    struct MultiDetectionInfo multi_detection_info;
    multi_detection_info.mode = msg->mode;
    multi_detection_info.num_objs = msg->num_objs;
    for(int i = 0; i < msg->num_objs; i++)
    {
        struct DetectionInfo detection_info;
        detection_info.left = msg->detection_infos[i].left;
        detection_info.top = msg->detection_infos[i].top;
        detection_info.bot = msg->detection_infos[i].bot;
        detection_info.right = msg->detection_infos[i].right;
        detection_info.trackIds = msg->detection_infos[i].trackIds;
        multi_detection_info.detection_infos.push_back(detection_info);
    }
    this->communication_->sendMsgByUdp(this->communication_->encodeMsg(Send_Mode::UDP,multi_detection_info),this->multicast_udp_ip);
}