#ifndef SWARM_CONTROL_H
#define SWARM_CONTROL_H

#include <ros/ros.h>
#include <bitset>
#include <Eigen/Eigen>
#include <unistd.h>

#include <std_msgs/Bool.h>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/MavrosInterface.h>

#include "printf_utils.h"
#include "formation_utils.h"

using namespace std;

// 宏定义
#define SWARM_CONTROL_BOSHEN97_NUM_POINT 2 // 打印小数点
#define SWARM_CONTROL_BOSHEN97_MAX_AGENT_NUM 30

class SwarmControl
{
public:
    SwarmControl(ros::NodeHandle &nh_);

    void mainLoop();

    // 订阅话题
    ros::Subscriber uav_state_sub_;
    ros::Subscriber swarm_command_sub_;
    // 发布话题
    ros::Publisher uav_cmd_pub_;
    ros::Publisher mavros_interface_pub_;
    // 定时器
    ros::Timer debug_timer_;

    // 变量
    //agent:智能体
    int agent_id_;
    string node_name_;
    float mainloop_rate_;
    ros::Time last_process_time_;
    float setmode_timeout_;
    int stop_mode_;
    float takeoff_height_;

    // 数量
    int uav_num_;

    struct FormationVelControlParam
    {
        float k_aij;
        float k_p;
        float k_gamma;
        float APF_R;
        float APF_r;
    };

    FormationVelControlParam vel_ctrl_param_;

    // follow
    float virtual_leader_pos_[3];
    float virtual_leader_vel_[2];
    Eigen::MatrixXf formation_separation_; // 阵型偏移量

    // 集群控制指令
    prometheus_msgs::SwarmCommand swarm_command_;
    prometheus_msgs::SwarmCommand swarm_command_last_;
    // 待发布的底层控制指令
    prometheus_msgs::UAVCommand uav_command_;
    prometheus_msgs::MavrosInterface mavros_interface_;

    // 本机状态
    prometheus_msgs::UAVState uav_state_;
    Eigen::Vector3d uav_pos_;
    Eigen::Vector3d uav_vel_;

private:
    void uavStateCb(const prometheus_msgs::UAVState::ConstPtr &msg);
    void swarmCommandCb(const prometheus_msgs::SwarmCommand::ConstPtr &msg);
    void debugCb(const ros::TimerEvent &e);

    void formationWithPositionControl();
    void formationWithVelocityControl();
    void printfParam();
};

#endif