// ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>

// topic 头文件
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVControlState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include "message_utils.h"

#include "printf_utils.h"

using namespace std;
using namespace Eigen;
#define NODE_NAME "object_tracking"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::UAVState g_UAVState;
Eigen::Vector3f g_drone_pos;
//---------------------------------------Vision---------------------------------------------
prometheus_msgs::DetectionInfo g_Detection_raw;      //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
prometheus_msgs::UAVControlState g_uavcontrol_state; //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
Eigen::Vector3f pos_body_frame;
Eigen::Vector3f pos_body_enu_frame;    //原点位于质心，x轴指向前方，y轴指向左，z轴指向上的坐标系
float kpx_track, kpy_track, kpz_track; //控制参数 - 比例参数
bool is_detected = false;              // 是否检测到目标标志
int num_count_vision_lost = 0;         //视觉丢失计数器
int num_count_vision_regain = 0;       //视觉丢失计数器
int g_uav_id;
int Thres_vision = 0; //视觉丢失计数器阈值
Eigen::Vector3f camera_offset;
//---------------------------------------Track---------------------------------------------
float distance_to_setpoint;
Eigen::Vector3f tracking_delta;
//---------------------------------------Output---------------------------------------------
prometheus_msgs::UAVCommand g_command_now; //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vision_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    g_Detection_raw = *msg;

    pos_body_frame[0] = g_Detection_raw.position[2] + camera_offset[0];
    pos_body_frame[1] = -g_Detection_raw.position[0] + camera_offset[1];
    pos_body_frame[2] = -g_Detection_raw.position[1] + camera_offset[2];

    Eigen::Matrix3f R_Body_to_ENU;

    R_Body_to_ENU = get_rotation_matrix(g_UAVState.attitude[0], g_UAVState.attitude[1], g_UAVState.attitude[2]);

    pos_body_enu_frame = R_Body_to_ENU * pos_body_frame;

    if (g_Detection_raw.detected)
    {
        num_count_vision_regain++;
        num_count_vision_lost = 0;
    }
    else
    {
        num_count_vision_regain = 0;
        num_count_vision_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if (num_count_vision_lost > Thres_vision)
    {
        is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if (num_count_vision_regain > Thres_vision)
    {
        is_detected = true;
    }
}
void drone_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    g_UAVState = *msg;

    g_drone_pos[0] = g_UAVState.position[0];
    g_drone_pos[1] = g_UAVState.position[1];
    g_drone_pos[2] = g_UAVState.position[2];
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_tracking");
    ros::NodeHandle nh("~");

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    //视觉丢失次数阈值
    nh.param<int>("Thres_vision", Thres_vision, 10);

    //追踪的前后间隔
    nh.param<float>("tracking_delta_x", tracking_delta[0], 2);
    nh.param<float>("tracking_delta_y", tracking_delta[1], 0.0);
    nh.param<float>("tracking_delta_z", tracking_delta[2], 2);

    nh.param<float>("camera_offset_x", camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", camera_offset[2], 0.0);

    //追踪控制参数
    nh.param<float>("kpx_track", kpx_track, 0.1);
    nh.param<float>("kpy_track", kpy_track, 0.1);
    nh.param<float>("kpz_track", kpz_track, 0.1);

    nh.param<int>("uav_id", g_uav_id, 1);

    // 订阅视觉反馈
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/uav" + std::to_string(g_uav_id) + "/prometheus/object_detection/siamrpn_tracker", 10, vision_cb);
    // 订阅当前无人机控制状态, 使用lambda匿名函数
    ros::Subscriber uav_control_state_sub = nh.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(g_uav_id) + "/prometheus/control_state", 10, [&](const prometheus_msgs::UAVControlState::ConstPtr &msg) -> void
                                                                                           { g_uavcontrol_state = *msg; });
    // 订阅无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(g_uav_id) + "/prometheus/state", 10, drone_state_cb);

    // 发布控制指令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(g_uav_id) + "/prometheus/command", 10);

    // 初始化
    g_command_now.position_ref[0] = 0;
    g_command_now.position_ref[1] = 0;
    g_command_now.position_ref[2] = 0;

    bool is_inited = false;

    while (ros::ok())
    {
        // 执行回调获取数据
        ros::spinOnce();
        if (g_uavcontrol_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, TAIL, "Waiting for enter COMMAND_CONTROL state");
            continue;
        }

        if (!is_detected)
        {
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            PCOUT(-1, GREEN, "Waiting for detected object");
        }
        else
        {
            // 到目标的直线距离(估计值)
            distance_to_setpoint = pos_body_frame.norm();

            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            // 使用全速度控制
            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL; // xy velocity z position

            // 根据误差计算计算应该给于无人机的速度
            g_command_now.velocity_ref[0] = kpx_track * (pos_body_enu_frame[0] - tracking_delta[0]);
            g_command_now.velocity_ref[1] = kpy_track * (pos_body_enu_frame[1] - tracking_delta[1]);
            g_command_now.velocity_ref[2] = kpz_track * (tracking_delta[2] - g_UAVState.position[2]);
            g_command_now.yaw_ref = 0;
            std::string tmp = "[object_tracking]: Tracking the Target, distance_to_setpoint : " + std::to_string(distance_to_setpoint) + " [m] " + "\n velocity_x: " + std::to_string(g_command_now.velocity_ref[0]) + "  [m/s], velocity_y: " + std::to_string(g_command_now.velocity_ref[1]) + " [m/s], velocity_y: " + std::to_string(g_command_now.velocity_ref[2]) + " [m/s]";
            PCOUT(1, GREEN, tmp);
        }

        // Publish
        g_command_now.header.stamp = ros::Time::now();
        g_command_now.Command_ID = g_command_now.Command_ID + 1;
        if (g_command_now.Command_ID < 10)
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        command_pub.publish(g_command_now);

        rate.sleep();
    }

    return 0;
}
