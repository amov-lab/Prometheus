#include "autonomous_landing_topic.hpp"

AutonomousLanding::AutonomousLanding(ros::NodeHandle &nh,Communication *communication)
{
    nh.param<int>("ROBOT_ID", robot_id, 0);
    nh.param<std::string>("multicast_udp_ip", multicast_udp_ip, "224.0.0.88");

    this->communication_ = communication;

    //prefix.c_str() + std::to_string(robot_id) + 
    //【服务】是否开启搜索
    this->gimbal_search_client_ = nh.serviceClient<std_srvs::SetBool>("/gimbal/search");
    //【服务】是否开启视频录制
    this->gimbal_record_video_client_ = nh.serviceClient<std_srvs::SetBool>("/gimbal/record_video");
    //【服务】是否自动反馈(真实IMU速度)
    this->gimbal_track_mode_client_ = nh.serviceClient<std_srvs::SetBool>("/gimbal/track_mode");
    //【服务】自主降落参数配置
    this->param_set_client_ = nh.serviceClient<mavros_msgs::ParamSet>("/autonomous_landing/ParamSet");
    //【发布】无人车数据
    // this->ugv_state_pub_ = nh.advertise<prometheus_msgs::RheaState>("/ugv1/prometheus/state", 1000);    
};

AutonomousLanding::~AutonomousLanding()
{
    // delete this->communication_;
};

void AutonomousLanding::gimbalSearchServer(bool is)
{
    std_srvs::SetBool set_bool;
    set_bool.request.data = is;
    
    this->gimbal_search_client_.call(set_bool);
}

void AutonomousLanding::gimbalRecordVideoServer(bool is)
{
    std_srvs::SetBool set_bool;
    set_bool.request.data = is;
    this->gimbal_record_video_client_.call(set_bool);
}

void AutonomousLanding::gimbalTrackModeServer(bool is)
{
    std_srvs::SetBool set_bool;
    set_bool.request.data = is;
    this->gimbal_track_mode_client_.call(set_bool);
}

void AutonomousLanding::gimbalParamSetServer(struct GimbalParamSet param_set)
{
    mavros_msgs::ParamSet srv;
    srv.request.param_id = param_set.param_id;
    srv.request.value.real = param_set.real;
    this->param_set_client_.call(srv);
}
