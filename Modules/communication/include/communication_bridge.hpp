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

#include "custom_data_segment.hpp"

//uav control
// #define OPENUAVBASIC ""//"gnome-terminal -- roslaunch prometheus_uav_control uav_control_main_indoor.launch"
// #define CLOSEUAVBASIC ""//"gnome-terminal -- rosnode kill /joy_node | gnome-terminal -- rosnode kill /uav_control_main_1"
//rhea control
// #define OPENUGVBASIC ""
// #define CLOSEUGVBASIC ""
//集群
// #define OPENSWARMCONTROL ""
// #define CLOSESWARMCONTROL ""
//自主降落
#define OPENAUTONOMOUSLANDING ""
#define CLOSEAUTONOMOUSLANDING ""
//目标识别与追踪
#define OPENOBJECTTRACKING ""
#define CLOSEOBJECTTRACKING ""
//EGO Planner
#define OPENEGOPLANNER ""
#define CLOSEEGOPLANNER ""

//杀掉除了通信节点和主节点的其他节点
//分为两种情况  
//1:杀掉子模块，这种情况不会杀掉uav control节点和通信节点以及master节点。 
//2:杀掉uav control节点，这种情况下只会保留通信节点以及master节点。
// #define CLOSEUAVBASIC "gnome-terminal -- rosnode kill `rosnode list | grep -v /communication_bridge | grep -v /rosout`"
#define CLOSEOTHERMODE "gnome-terminal -- rosnode kill `rosnode list | grep -v /communication_bridge | grep -v /rosout | grep -v /uav_control_main_1 | grep -v /joy_node`"

//重启
#define REBOOTNXCMD "shutdown -r now"
//关机
#define EXITNXCMD "shutdown -h now"

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
    void recvData(struct UGVCommand ugv_command);
    void recvData(struct UGVState ugv_state);
    void recvData(struct ImageData image_data);
    void recvData(struct UAVSetup uav_setup);
    void recvData(struct ModeSelection mode_selection);
    void recvData(struct ParamSettings param_settings);
    void recvData(struct MultiBsplines multi_bsplines);
    void recvData(struct Bspline bspline);
    void recvData(struct CustomDataSegment_1 custom_data_segment);
    // void recvData(struct CustomDataSegment_2 custom_data_segment);
    void recvData(struct Goal goal);

    void modeSwitch(struct ModeSelection mode_selection);

    bool getParam(struct Param *param);

    void sendControlParam();
    void sendCommunicationParam();
    void sendSwarmParam();
    void sendCommandPubParam();

    void sendTextInfo(uint8_t message_type, std::string message);

    //接收组播地址的数据
    void multicastUdpFun();

    //给地面站发送心跳包
    void toGroundStationFun();

    void createImage(struct ImageData image_data);

    void createMode(struct ModeSelection mode_selection);

    void deleteMode(struct ModeSelection mode_selection);

    template <typename T>
    bool setParam(std::string param_name, T param_value);

    void checkHeartbeatState(const ros::TimerEvent& time_event);
    void toGroundHeartbeat(const ros::TimerEvent& time_event);

    //触发安全机制处理
    void triggerUAV();
    void triggerSwarmControl();
    void triggerUGV();

private:
    // std::shared_ptr<SwarmControl> swarm_control_ ;
    SwarmControl *swarm_control_ = NULL;
    UGVBasic *ugv_basic_ = NULL;
    UAVBasic *uav_basic_ = NULL;
    // std::vector<UAVBasic*> swarm_control_simulation_;
    std::map<int, UAVBasic *> swarm_control_simulation_;
    std::map<int, UGVBasic *> swarm_ugv_control_simulation_;
    AutonomousLanding *autonomous_landing_ = NULL;
    GimbalBasic *gimbal_basic_ = NULL;
    ObjectTracking *object_tracking_ = NULL;

    EGOPlannerSwarm *ego_planner_ = NULL;
    EGOPlannerSwarm *trajectoy_control_ = NULL;

    int current_mode_ = 0;

    int is_simulation_, swarm_num_, swarm_data_update_timeout_,swarm_ugv_num_;
    int uav_id,ugv_id;
    ros::NodeHandle nh_;

    bool is_heartbeat_ready_ = false;

    std::string OPENUAVBASIC = "", CLOSEUAVBASIC = "", OPENSWARMCONTROL = "", CLOSESWARMCONTROL = "",OPENUGVBASIC = "" , CLOSEUGVBASIC = "";

    ros::Timer heartbeat_check_timer;
    bool disconnect_flag = false;
    long heartbeat_count = 0;

    // 心跳包数据
    ros::Timer to_ground_heartbeat_timer;
    struct Heartbeat heartbeat;
    // 记录 无人机或无人车的时间戳
    uint time = 0;
    uint time_count = 0;
    std::vector<uint> swarm_control_time;
    std::vector<uint> swarm_control_timeout_count;
};

template <typename T>
bool CommunicationBridge::setParam(std::string param_name, T param_value)
{
    if (nh_.hasParam(param_name))
    {
        nh_.setParam(param_name, param_value);
        T value;
        nh_.getParam(param_name, value);
        if (param_value == value)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

#endif