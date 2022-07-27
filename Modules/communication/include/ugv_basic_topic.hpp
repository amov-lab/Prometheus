#ifndef UGV_BASIC_TOPIC_HPP
#define UGV_BASIC_TOPIC_HPP

#include "communication.hpp"
#include "prometheus_msgs/RheaCommunication.h"
#include "prometheus_msgs/RheaState.h"
#include "prometheus_msgs/RheaGPS.h"
using namespace std;

class UGVBasic
{
public:
    UGVBasic(ros::NodeHandle &nh);

    ~UGVBasic();

    void scanCb(const sensor_msgs::LaserScan::ConstPtr &msg);

    void scanMatchedPoints2Cb(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void tfCb(const tf2_msgs::TFMessage::ConstPtr &msg);

    void tfStaticCb(const tf2_msgs::TFMessage::ConstPtr &msg);

    void constraintListCb(const visualization_msgs::MarkerArray::ConstPtr &msg);

    void landmarkPosesListCb(const visualization_msgs::MarkerArray::ConstPtr &msg);

    void trajectoryNodeListCb(const visualization_msgs::MarkerArray::ConstPtr &msg);

    void rheaControlPub(struct RheaControl rhea_control);

    void rheaStateCb(const prometheus_msgs::RheaState::ConstPtr &msg);

    void setTimeStamp(uint time);

    uint getTimeStamp();

private:
    //rviz
    ros::Subscriber scan_matched_points2_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber tf_static_sub_;
    ros::Subscriber tf_sub_;
    ros::Subscriber constraint_list_sub_;
    ros::Subscriber landmark_poses_list_sub_;
    ros::Subscriber trajectory_node_list_sub_;
    //
    ros::Publisher rhea_control_pub_;
    ros::Subscriber rhea_state_sub_;

    ros::Subscriber cmd_vel_sub_;

    Communication* communication_ = NULL;

    std::string udp_ip;
    std::string multicast_udp_ip;

    uint time_stamp_ = 0;
};

#endif