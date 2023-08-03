#include "ugv_basic_topic.hpp"

UGVBasic::UGVBasic(ros::NodeHandle &nh,Communication *communication)
{
    this->communication_ = communication;

    nh.param<int>("ROBOT_ID", robot_id, 1);
    nh.param<std::string>("ground_station_ip", udp_ip, "127.0.0.1");
    nh.param<std::string>("multicast_udp_ip", multicast_udp_ip, "224.0.0.88");

    this->ugv_cmd_pub_ = nh.advertise<prometheus_msgs::UGVCommand>("/ugv" + to_string(robot_id) + "/prometheus/ugv_command", 1000);
    this->ugv_state_sub_ = nh.subscribe("/ugv" + to_string(robot_id) + "/prometheus/ugv_state", 10, &UGVBasic::stateCb, this);
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
    for(int i = 0; i < 2; i++)
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
    struct UGVState ugv_state;
    ugv_state.ugv_id = msg->ugv_id;
    ugv_state.secs = msg->header.stamp.sec;
    ugv_state.nsecs = msg->header.stamp.nsec;
    ugv_state.battery = msg->battery;
    for(int i = 0; i < 3; i++)
    {
        ugv_state.position[i] = msg->position[i];
        ugv_state.velocity[i] = msg->velocity[i];
        ugv_state.attitude[i] = msg->attitude[i];
    }
    ugv_state.attitude_q.x = msg->attitude_q.x;
    ugv_state.attitude_q.y = msg->attitude_q.y;
    ugv_state.attitude_q.z = msg->attitude_q.z;
    ugv_state.attitude_q.w = msg->attitude_q.w;
    
    this->communication_->sendMsgByUdp(this->communication_->encodeMsg(Send_Mode::UDP,ugv_state),multicast_udp_ip);
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