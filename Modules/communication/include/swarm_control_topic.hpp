#ifndef SWARM_CONTROL_TOPIC_HPP
#define SWARM_CONTROL_TOPIC_HPP

#include <ros/ros.h>
#include "communication.hpp"
#include "uav_basic_topic.hpp"

#include "std_msgs/Bool.h"

#include "prometheus_msgs/MultiUAVState.h"
#include "prometheus_msgs/SwarmCommand.h"
#include "prometheus_msgs/UAVState.h"
#include "prometheus_msgs/OffsetPose.h"

#include <vector>

using namespace std;

//订阅: /prometheus/formation_assign/result
//发布: /Prometheus/formation_assign/info

struct MultiUAVState
{
    int uav_num;
    std::vector<UAVState> uav_state_all;
};

class SwarmControl//: public UAVBasic
{
public:
    //真机构造
    SwarmControl(ros::NodeHandle &nh, int id, int swarm_num,Communication *communication);

    //仿真构造
    SwarmControl(ros::NodeHandle &nh, int swarm_num,Communication *communication);

    ~SwarmControl();

    void init(ros::NodeHandle &nh, int swarm_num,int id = 1);

    //更新全部飞机数据
    void updateAllUAVState(struct UAVState uav_state);

    //【发布】集群控制指令
    void swarmCommandPub(struct SwarmCommand swarm_command);

    //【发布】连接是否失效
    void communicationStatePub(bool communication);

    void communicationStatePub(bool communication,int id);

    //【发布】所有无人机状态
    void allUAVStatePub(struct MultiUAVState m_multi_uav_state);

    void swarmCmdCb(const prometheus_msgs::SwarmCommand::ConstPtr &msg);

    inline struct MultiUAVState getMultiUAVState(){return this->multi_uav_state_;};

    inline prometheus_msgs::UAVState getUAVStateMsg(){return this->uav_state_msg_;};


private:

    struct MultiUAVState multi_uav_state_;

    Communication *communication_ = NULL;

    prometheus_msgs::UAVState uav_state_msg_;

    
    //集群全部uav 状态
    ros::Publisher all_uav_state_pub_;
    //控制指令
    ros::Publisher swarm_command_pub_;
    //连接是否失效
    ros::Publisher communication_state_pub_;

    ros::Subscriber swarm_command_sub_;


    //仿真
    std::vector<ros::Publisher> simulation_communication_state_pub;

    bool is_simulation_;

    std::string udp_ip, multicast_udp_ip;

    std::string user_type_ = "";
};

#endif