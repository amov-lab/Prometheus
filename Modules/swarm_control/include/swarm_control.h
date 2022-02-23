#ifndef SWARM_CONTROL_H
#define SWARM_CONTROL_H

#include <ros/ros.h>
#include <bitset>
#include <Eigen/Eigen>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/MultiUAVState.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/SwarmCommand.h>

#include "printf_utils.h"
#include "formation_utils.h"

using namespace std;

// 宏定义
#define NUM_POINT 2 // 打印小数点
#define MAX_AGENT_NUM 30

class SwarmControl
{
public:
    SwarmControl(ros::NodeHandle &nh);

    void mainloop();

    // 订阅话题
    ros::Subscriber uav_state_sub;
    ros::Subscriber all_uav_state_sub;
    ros::Subscriber swarm_command_sub;
    // 发布话题
    ros::Publisher uav_cmd_pub;
    // 定时器
    ros::Timer debug_timer;

    // 变量
    int agent_id;
    string node_name;
    float mainloop_rate;
    ros::Time last_process_time;

    // 数量
    int uav_num;

    float safe_dis_to_land{0.4};
    float safe_dis_to_hold{0.8};

    struct formation_vel_control_param
    {
        float k_aij;
        float k_p;
        float k_gamma;
        float APF_R;
        float APF_r;
    };
    formation_vel_control_param vel_ctrl_param;

    // follow
    float virtual_leader_pos[2];
    float virtual_leader_vel[2];
    Eigen::MatrixXf formation_separation; // 阵型偏移量

    // 集群控制指令
    prometheus_msgs::SwarmCommand swarm_command;
    prometheus_msgs::SwarmCommand swarm_command_last;
    // 待发布的底层控制指令
    prometheus_msgs::UAVCommand uav_command;

    // 本机状态
    prometheus_msgs::UAVState uav_state;
    Eigen::Vector3d uav_pos;
    Eigen::Vector3d uav_vel;
    int collision_flag;

    // 所有agent的状态
    prometheus_msgs::UAVState uav_states[MAX_AGENT_NUM + 1];
    Eigen::Vector3d all_uav_pos[MAX_AGENT_NUM + 1];
    Eigen::Vector3d all_uav_vel[MAX_AGENT_NUM + 1];

private:
    void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg);
    void all_uav_state_cb(const prometheus_msgs::MultiUAVState::ConstPtr &msg);
    void swarm_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr &msg);
    void debug_cb(const ros::TimerEvent &e);

    int check_collision();
    void checkUAVState();
    void formation_with_position_control();
    void formation_with_velocity_control();
    void printf_param();
};

#endif