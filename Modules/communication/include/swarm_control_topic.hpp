#ifndef SWARM_CONTROL_TOPIC_HPP
#define SWARM_CONTROL_TOPIC_HPP

#include <ros/ros.h>
#include "communication.hpp"
#include "uav_basic_topic.hpp"
#include "ugv_basic_topic.hpp"

#include "std_msgs/Bool.h"

#include "prometheus_msgs/MultiUAVState.h"
#include "prometheus_msgs/SwarmCommand.h"
#include "prometheus_msgs/UAVState.h"
#include "prometheus_msgs/OffsetPose.h"
#include "prometheus_msgs/MultiUGVState.h"

#include <vector>

using namespace std;

//订阅: /prometheus/formation_assign/result
//发布: /Prometheus/formation_assign/info

struct MultiUAVState
{
    int uav_num;
    std::vector<UAVState> uav_state_all;
};

struct MultiUGVState
{
    int ugv_num;
    std::vector<UGVState> ugv_state_all;
};

enum SwarmMode
{
    ONLY_UAV = 1,
    ONLY_UGV = 2,
    UAV_AND_UGV = 3
};

// 集群模式-载体
enum RobotType
{
    ROBOT_TYPE_UAV = 1,
    ROBOT_TYPE_UGV = 2,
    ROBOT_TYPE_SIMULATION = 3
};

class SwarmControl//: public UAVBasic
{
public:
    // 真机构造函数
    // 真机：无人机或者无人车集群
    /**
     * @brief 构造函数，真机集群：无人机集群、无人车集群
     * 
     * @param nh ros节点句柄
     * @param communication 通信模块
     * @param mode 集群模式，枚举值
     * @param id 当前robot_id
     * @param swarm_num 集群数量
     */
    SwarmControl(ros::NodeHandle &nh, Communication *communication, SwarmMode mode, int id, int swarm_num);
    // 真机：机车协同
    /**
     * @brief 构造函数，真机集群：机车协同
     * 
     * @param nh ros节点句柄
     * @param communication 通信模块
     * @param mode 集群模式，枚举值
     * @param type 机器类型，枚举值
     * @param id 当前robot_id
     * @param swarm_uav_num 集群中无人机数量
     * @param swarm_ugv_num 集群中无人车数量
     */
    SwarmControl(ros::NodeHandle &nh, Communication *communication, SwarmMode mode, RobotType type, int id, int swarm_uav_num, int swarm_ugv_num);

    // 仿真构造函数
    // 仿真：无人机或者无人车集群、机车协同
    /**
     * @brief 构造函数，仿真集群：无人机或者无人车集群、机车协同
     * 
     * @param nh ros节点句柄
     * @param communication 通信模块
     * @param mode 集群模式，枚举值
     * @param type 机器类型，枚举值
     * @param swarm_uav_num 集群中无人机数量
     * @param swarm_ugv_num 集群中无人车数量
     */
    SwarmControl(ros::NodeHandle &nh, Communication *communication, SwarmMode mode, RobotType type, int swarm_uav_num, int swarm_ugv_num);

    ~SwarmControl();

    void init(ros::NodeHandle &nh, SwarmMode mode, int swarm_num);
    void init(ros::NodeHandle &nh, SwarmMode mode, int swarm_uav_num,int swarm_ugv_num);

    // 更新全部无人机数据
    void updateAllUAVState(struct UAVState uav_state);

    // 更新全部无人车数据
    void updateAllUGVState(struct UGVState ugv_state);

    //【发布】集群控制指令
    void swarmCommandPub(struct SwarmCommand swarm_command);

    //【发布】连接是否失效
    void communicationStatePub(bool communication);

    void communicationStatePub(bool communication,int id);

    //【发布】所有无人机状态
    void allUAVStatePub(struct MultiUAVState m_multi_uav_state);

    //【发布】所有无人车状态
    void allUGVStatePub(struct MultiUGVState m_multi_ugv_state);

    void swarmCmdCb(const prometheus_msgs::SwarmCommand::ConstPtr &msg);

    inline struct MultiUAVState getMultiUAVState(){return this->multi_uav_state_;};

    inline struct MultiUGVState getMultiUGVState(){return this->multi_ugv_state_;};

    inline prometheus_msgs::UAVState getUAVStateMsg(){return this->uav_state_msg_;};

private:

    struct MultiUAVState multi_uav_state_;

    struct MultiUGVState multi_ugv_state_;

    Communication *communication_ = NULL;

    prometheus_msgs::UAVState uav_state_msg_;

    
    //集群全部uav 状态
    ros::Publisher all_uav_state_pub_;
    //集群全部ugv 状态
    ros::Publisher all_ugv_state_pub_;
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