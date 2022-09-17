#ifndef COMUNICATION_BRIDGE_HPP
#define COMUNICATION_BRIDGE_HPP

#include <ros/ros.h>
#include "communication.hpp"

#include <boost/thread.hpp>

#include <thread>
#include "Struct.hpp"

#include "uav_basic_topic.hpp"
#include "ugv_basic_topic.hpp"
#include "swarm_control_topic.hpp"
#include "autonomous_landing_topic.hpp"
#include "gimbal_basic_topic.hpp"
#include "object_tracking_topic.hpp"
#include "ego_planner_swarm_topic.hpp"

#include <mutex>
#include <condition_variable>

enum UserType
{
    UAV = 1,
    UGV = 2
};

class CommunicationBridge : public Communication
{
public:
    CommunicationBridge(ros::NodeHandle &nh);

    ~CommunicationBridge();

    void serverFun();

    //根据协议中MSG_ID的值，将数据段数据转化为正确的结构体
    void pubMsg(int msg_id);

    void recvData(struct UAVState uav_state);
    void recvData(struct UAVCommand uav_cmd);
    void recvData(struct SwarmCommand swarm_command);
    void recvData(struct ConnectState connect_state);
    void recvData(struct GimbalControl gimbal_control);
    void recvData(struct GimbalService gimbal_service);
    void recvData(struct GimbalParamSet param_set);
    void recvData(struct WindowPosition window_position);
    void recvData(struct RheaControl rhea_control);
    void recvData(struct RheaState rhea_state);
    void recvData(struct ImageData image_data);
    void recvData(struct UAVSetup uav_setup);
    void recvData(struct ModeSelection mode_selection);
    void recvData(struct ParamSettings param_settings);
    void recvData(struct MultiBsplines multi_bsplines);
    void recvData(struct Bspline bspline);

    void modeSwitch(struct ModeSelection mode_selection);

    //接收组播地址的数据
    void multicastUdpFun();

    //给地面站发送心跳包
    void toGroundStationFun();

    void init();
    //ros::NodeHandle nh;

    void createImage(struct ImageData image_data);

    bool createMode(struct ModeSelection mode_selection);

    bool deleteMode(struct ModeSelection mode_selection);

    template <typename T>
    bool setParam(std::string param_name,T param_value);
private:
    //std::shared_ptr<SwarmControl> swarm_control_ ;
    SwarmControl *swarm_control_ = NULL;
    UGVBasic *ugv_basic_ = NULL;
    UAVBasic *uav_basic_ = NULL;
    //std::vector<UAVBasic*> swarm_control_simulation_;
    std::map<int,UAVBasic*> swarm_control_simulation_;
    AutonomousLanding *autonomous_landing_ = NULL;
    GimbalBasic *gimbal_basic_ = NULL;
    ObjectTracking *object_tracking_ = NULL;
    EGOPlannerSwarm *ego_planner_ = NULL;

    int current_mode_ = 0;

    int is_simulation_, swarm_num_, swarm_data_update_timeout_;
    ros::NodeHandle nh_;

    bool is_heartbeat_ready_ = false;

    int user_type_;
};

template <typename T>
bool CommunicationBridge::setParam(std::string param_name,T param_value)
{
    nh_.setParam(param_name,param_value);
    T value;
    nh_.getParam(param_name,value);
    if(param_value == value)
    {
        return true;
    }else
    {
        return false;
    }
    return false;
}

#endif