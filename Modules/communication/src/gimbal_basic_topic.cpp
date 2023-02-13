#include "gimbal_basic_topic.hpp"

GimbalBasic::GimbalBasic(ros::NodeHandle &nh,Communication *communication)
{
    nh.param<std::string>("multicast_udp_ip", multicast_udp_ip, "224.0.0.88");
    int id;
    nh.param<int>("ROBOT_ID", id, 1);
    this->communication_ = communication;
    //【订阅】吊舱状态数据
    this->gimbal_state_sub_ = nh.subscribe("/uav" + std::to_string(id) + "/gimbal/state", 10, &GimbalBasic::stateCb, this);
    //【订阅】跟踪误差
    this->vision_diff_sub_ = nh.subscribe("/uav" + std::to_string(id) + "/gimbal/track", 10, &GimbalBasic::trackCb, this);
    //【发布】框选 点击 目标
    this->window_position_pub_ = nh.advertise<prometheus_msgs::WindowPosition>("/detection/bbox_draw", 1000);
    //【发布】吊舱控制
    this->gimbal_control_pub_ = nh.advertise<prometheus_msgs::GimbalControl>("/uav" + std::to_string(id) + "/gimbal/control", 1000);
}

GimbalBasic::~GimbalBasic()
{
    // delete this->communication_;
}

void GimbalBasic::stateCb(const prometheus_msgs::GimbalState::ConstPtr &msg)
{
    this->gimbal_state_.Id = msg->Id;
    this->gimbal_state_.feedbackMode = msg->feedbackMode;
    this->gimbal_state_.mode = msg->mode;
    this->gimbal_state_.isRecording = msg->isRecording;
    this->gimbal_state_.zoomState = msg->zoomState;
    this->gimbal_state_.zoomVal = msg->zoomVal;
    for(int i = 0; i < 3; i++)
    {
        this->gimbal_state_.imuAngle[i] = msg->imuAngle[i];
        this->gimbal_state_.rotorAngle[i] = msg->rotorAngle[i];
        this->gimbal_state_.imuAngleVel[i] = msg->imuAngleVel[i];
        this->gimbal_state_.rotorAngleTarget[i] = msg->rotorAngleTarget[i];
    }
    //发送到组播地址
    this->communication_->sendMsgByUdp(this->communication_->encodeMsg(Send_Mode::UDP,this->gimbal_state_),multicast_udp_ip);
}

void GimbalBasic::trackCb(const prometheus_msgs::VisionDiff::ConstPtr &msg)
{
    this->vision_diff_.id = msg->Id;
    this->vision_diff_.detect = msg->detect;
    this->vision_diff_.objectX = msg->objectX;
    this->vision_diff_.objectY = msg->objectY;
    this->vision_diff_.objectWidth = msg->objectWidth;
    this->vision_diff_.objectHeight = msg->objectHeight;
    this->vision_diff_.frameWidth = msg->frameWidth;
    this->vision_diff_.frameHeight = msg->frameHeight;
    this->vision_diff_.kp = msg->kp;
    this->vision_diff_.ki = msg->ki;
    this->vision_diff_.kd = msg->kd;
    this->vision_diff_.ctlMode = msg->ctlMode;
    this->vision_diff_.currSize = msg->currSize;
    this->vision_diff_.maxSize = msg->maxSize;
    this->vision_diff_.minSize = msg->minSize;
    this->vision_diff_.trackIgnoreError = msg->trackIgnoreError;
    this->vision_diff_.autoZoom = msg->autoZoom;
    //发送到组播地址
    this->communication_->sendMsgByUdp(this->communication_->encodeMsg(Send_Mode::UDP,this->vision_diff_),multicast_udp_ip);
}

void GimbalBasic::gimbalWindowPositionPub(struct WindowPosition window_position)
{
    prometheus_msgs::WindowPosition window_position_;
    window_position_.mode = window_position.mode;
    window_position_.track_id = window_position.track_id;
    window_position_.origin_x = window_position.origin_x;
    window_position_.origin_y = window_position.origin_y;
    window_position_.height = window_position.height;
    window_position_.width = window_position.width;
    window_position_.window_position_x = window_position.window_position_x;
    window_position_.window_position_y = window_position.window_position_y;
    this->window_position_pub_.publish(window_position_);
}

void GimbalBasic::gimbalControlPub(struct GimbalControl gimbal_control)
{
    prometheus_msgs::GimbalControl gimbal_control_;
    gimbal_control_.Id = gimbal_control.Id;
    gimbal_control_.rpyMode = gimbal_control.rpyMode;
    gimbal_control_.roll = gimbal_control.roll;
    gimbal_control_.yaw = gimbal_control.yaw;
    gimbal_control_.pitch = gimbal_control.pitch;
    gimbal_control_.rValue = gimbal_control.rValue;
    gimbal_control_.yValue = gimbal_control.yValue;
    gimbal_control_.pValue = gimbal_control.pValue;
    gimbal_control_.focusMode = gimbal_control.focusMode;
    gimbal_control_.zoomMode = gimbal_control.zoomMode;
    //发布话题
    this->gimbal_control_pub_.publish(gimbal_control_);
}
