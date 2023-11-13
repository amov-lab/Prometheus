#include "ugv_basic_topic.hpp"

UGVBasic::UGVBasic(ros::NodeHandle &nh, int id, Communication *communication)
{
    this->communication_ = communication;

    this->robot_id = -id;
    nh.param<std::string>("ground_station_ip", udp_ip, "127.0.0.1");
    nh.param<std::string>("multicast_udp_ip", multicast_udp_ip, "224.0.0.88");

    int offset;
    nh.param<int>("swarm_num", offset, 0);
    // this->robot_id = id + offset;

    this->ugv_cmd_pub_ = nh.advertise<prometheus_msgs::UGVCommand>("/ugv" + to_string(id) + "/prometheus/ugv_command", 1000);
    this->ugv_state_sub_ = nh.subscribe("/ugv" + to_string(id) + "/prometheus/ugv_state", 10, &UGVBasic::stateCb, this);
}

UGVBasic::~UGVBasic()
{
    // delete this->communication_;
}

void UGVBasic::ugvCmdPub(struct UGVCommand ugv_command)
{
    prometheus_msgs::UGVCommand ugv_command_;
    ugv_command_.header.stamp = ros::Time::now();
    ugv_command_.Command_ID = ugv_command.Command_ID;
    ugv_command_.Mode = ugv_command.Mode;
    for (int i = 0; i < 2; i++)
    {
        ugv_command_.position_ref[i] = ugv_command.position_ref[i];
        ugv_command_.linear_vel[i] = ugv_command.linear_vel[i];
    }
    ugv_command_.yaw_ref = ugv_command.yaw_ref;
    ugv_command_.angular_vel = ugv_command.angular_vel;
    this->ugv_cmd_pub_.publish(ugv_command_);
}

void UGVBasic::stateCb(const prometheus_msgs::UGVState::ConstPtr &msg)
{
    this->ugv_state_.ugv_id = msg->ugv_id;
    this->ugv_state_.secs = msg->header.stamp.sec;
    this->ugv_state_.nsecs = msg->header.stamp.nsec;
    this->ugv_state_.battery = msg->battery;
    for (int i = 0; i < 3; i++)
    {
        this->ugv_state_.position[i] = msg->position[i];
        this->ugv_state_.velocity[i] = msg->velocity[i];
        this->ugv_state_.attitude[i] = msg->attitude[i];
    }
    this->ugv_state_.attitude_q.x = msg->attitude_q.x;
    this->ugv_state_.attitude_q.y = msg->attitude_q.y;
    this->ugv_state_.attitude_q.z = msg->attitude_q.z;
    this->ugv_state_.attitude_q.w = msg->attitude_q.w;

    this->communication_->sendMsgByUdp(this->communication_->encodeMsg(Send_Mode::UDP, ugv_state_, robot_id), multicast_udp_ip);
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

struct UGVState UGVBasic::getUGVState()
{
    return this->ugv_state_;
}