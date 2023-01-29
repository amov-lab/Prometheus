#include "ugv_basic_topic.hpp"

UGVBasic::UGVBasic(ros::NodeHandle &nh,Communication *communication)
{
    this->communication_ = communication;
    nh.param<std::string>("ground_station_ip", udp_ip, "127.0.0.1");
    nh.param<std::string>("multicast_udp_ip", multicast_udp_ip, "224.0.0.88");

    //【订阅】rviz 点云
    // this->scan_matched_points2_sub_ = nh.subscribe("/prometheus/pcl_groundtruth", 10, &UGVBasic::scanMatchedPoints2Cb, this);
    // // //【订阅】rviz 激光雷达
    // this->scan_sub_ = nh.subscribe("/scan", 10, &UGVBasic::scanCb, this);
    // // //【订阅】rviz tf_static
    // this->tf_static_sub_ = nh.subscribe("/tf_static", 10, &UGVBasic::tfCb, this);
    // // //【订阅】rviz tf
    // this->tf_sub_ = nh.subscribe("/tf", 10, &UGVBasic::tfCb, this);
    //【订阅】rviz constraint_list
    //this->constraint_list_sub_
    //【订阅】rviz landmark_poses_list
    //this->landmark_poses_list_sub_
    //【订阅】rviz trajectory_node_list
    //this->trajectory_node_list_sub_
    this->rhea_control_pub_ = nh.advertise<prometheus_msgs::RheaCommunication>("/rhea_command/control", 1000);
    this->rhea_state_sub_ = nh.subscribe("/rhea_command/state", 10, &UGVBasic::rheaStateCb, this);
}

UGVBasic::~UGVBasic()
{
    // delete this->communication_;
}

// void UGVBasic::scanCb(const sensor_msgs::LaserScan::ConstPtr &msg)
// {
//     sensor_msgs::LaserScan laser_scan = *msg;
//     this->communication_->sendRvizByUdp(this->communication_->encodeRvizMsg(laser_scan), udp_ip);
// }

// void UGVBasic::scanMatchedPoints2Cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {
//     sensor_msgs::PointCloud2 scan_matched_points2 = *msg;
//     this->communication_->sendRvizByUdp(this->communication_->encodeRvizMsg(scan_matched_points2), udp_ip);
// }

// void UGVBasic::tfCb(const tf2_msgs::TFMessage::ConstPtr &msg)
// {
//     tf2_msgs::TFMessage tf = *msg;
//     this->communication_->sendRvizByUdp(this->communication_->encodeRvizMsg(tf), udp_ip);
// }

// void UGVBasic::tfStaticCb(const tf2_msgs::TFMessage::ConstPtr &msg)
// {
//     tf2_msgs::TFMessage tf_static = *msg;
//     this->communication_->sendRvizByUdp(this->communication_->encodeRvizMsg(tf_static, MsgId::UGVTFSTATIC), udp_ip);
// }

// void UGVBasic::constraintListCb(const visualization_msgs::MarkerArray::ConstPtr &msg)
// {
//     visualization_msgs::MarkerArray constraint_list = *msg;
//     this->communication_->sendRvizByUdp(this->communication_->encodeRvizMsg(constraint_list), udp_ip);
// }

// void UGVBasic::landmarkPosesListCb(const visualization_msgs::MarkerArray::ConstPtr &msg)
// {
//     visualization_msgs::MarkerArray landmark_poses_list = *msg;
//     this->communication_->sendRvizByUdp(this->communication_->encodeRvizMsg(landmark_poses_list, MsgId::UGVMARKERARRAYLANDMARK), udp_ip);
// }

// void UGVBasic::trajectoryNodeListCb(const visualization_msgs::MarkerArray::ConstPtr &msg)
// {
//     visualization_msgs::MarkerArray trajectory_node_list = *msg;
//     this->communication_->sendRvizByUdp(this->communication_->encodeRvizMsg(trajectory_node_list, MsgId::UGVMARKERARRAYTRAJECTORY), udp_ip);
// }

void UGVBasic::rheaControlPub(struct RheaControl rhea_control)
{
    prometheus_msgs::RheaCommunication rhea_control_;
    rhea_control_.Mode = rhea_control.Mode;
    rhea_control_.linear = rhea_control.linear;
    rhea_control_.angular = rhea_control.angular;
    for(int i = 0; i < rhea_control.waypoint.size(); i++)
    {
        prometheus_msgs::RheaGPS rhea_gps;
        rhea_gps.altitude = rhea_control.waypoint[i].altitude;
        rhea_gps.latitude = rhea_control.waypoint[i].latitude;
        rhea_gps.longitude = rhea_control.waypoint[i].longitude;
        rhea_control_.waypoint.push_back(rhea_gps);
    }
    this->rhea_control_pub_.publish(rhea_control_);
}

void UGVBasic::rheaStateCb(const prometheus_msgs::RheaState::ConstPtr &msg)
{
    struct RheaState rhea_state;
    rhea_state.rhea_id = msg->rhea_id;
    rhea_state.linear = msg->linear;
    rhea_state.angular = msg->angular;
    rhea_state.yaw = msg->yaw;
    rhea_state.latitude = msg->latitude;
    rhea_state.longitude = msg->longitude;
    rhea_state.battery_voltage = msg->battery_voltage;
    rhea_state.altitude = msg->altitude;
    for(int i = 0; i < 3; i++)
    {
        rhea_state.position[i] = msg->position[i];
    }
    this->communication_->sendMsgByUdp(this->communication_->encodeMsg(Send_Mode::UDP,rhea_state),multicast_udp_ip);
    setTimeStamp(msg->header.stamp.sec);
}

void UGVBasic::setTimeStamp(uint time)
{
    this->time_stamp_ = time;
}

uint UGVBasic::getTimeStamp()
{
    return this->time_stamp_;
}