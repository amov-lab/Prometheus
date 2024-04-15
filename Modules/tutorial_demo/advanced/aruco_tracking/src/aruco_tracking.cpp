#include <ros/ros.h>
#include <sstream>
#include <Eigen/Eigen>
#include <iostream>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVControlState.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/ROI.h>
#include <mission_utils.h>
#include "printf_utils.h"

using namespace std;
using namespace Eigen;
#define NODE_NAME "aruco_tracking"
prometheus_msgs::UAVState g_UAVState;
Eigen::Vector3f g_drone_pos;
prometheus_msgs::UAVControlState g_uavcontrol_state; //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
spirecv_msgs::Target g_Detection_raw;      //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
prometheus_msgs::UAVCommand g_command_now; //发送给控制模块的命令
Eigen::Vector3f pos_body_frame;
Eigen::Vector3f pos_body_enu_frame;    //原点位于质心，x轴指向前方，y轴指向左，z轴指向上的坐标系
float kpx_track, kpy_track, kpz_track; //控制参数 - 比例参数
bool is_detected = false;              // 是否检测到目标标志
int num_count_vision_lost = 0;         //视觉丢失计数器
int num_count_vision_regain = 0;       //视觉丢失计数器
int g_uav_id;
int Thres_vision = 0; //视觉丢失计数器阈值
Eigen::Vector3f camera_offset;
float distance_to_setpoint;
Eigen::Vector3f tracking_delta;
// 目标二维码id
int goto_id = 1;
void droneStateCb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    g_UAVState = *msg;
    g_drone_pos[0] = g_UAVState.position[0];
    g_drone_pos[1] = g_UAVState.position[1];
    g_drone_pos[2] = g_UAVState.position[2];
}

void VisionCb(const spirecv_msgs::TargetsInFrame::ConstPtr &msg)
{
    g_Detection_raw.mode = false;
    for(auto &target : msg->targets)
    {
        if(/*target.tracked_id != goto_id && */target.mode == false)
            continue;
        std::cout << "target aruco" << std::endl;
        g_Detection_raw = target;
    }
    // g_Detection_raw = *msg;
    pos_body_frame[0] = g_Detection_raw.pz + camera_offset[0];
    pos_body_frame[1] = -g_Detection_raw.px + camera_offset[1];
    pos_body_frame[2] = -g_Detection_raw.py + camera_offset[2];
    // pos_body_frame[0] = g_Detection_raw.px

    Eigen::Matrix3f R_Body_to_ENU;

    R_Body_to_ENU = get_rotation_matrix(g_UAVState.attitude[0], g_UAVState.attitude[1], g_UAVState.attitude[2]);

    pos_body_enu_frame = R_Body_to_ENU * pos_body_frame;


    if(g_Detection_raw.score == 1)
    {
        num_count_vision_regain++;
        num_count_vision_lost = 0;
    }else
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
inline bool checkInput(int &goto_id)
{
    std::cout << "Please input the number(1~20) for that point to land on, input 0 exit program" << std::endl;
    while (true)
    {
        try
        {
            std::cout << " > ";
            std::cin >> goto_id;
            if (goto_id == 0)
            {
                return false;
            }
            break;
        }
        catch (const std::exception &e)
        {
        }
        std::cout << "Input number not in the range 1 to 20, re-input" << std::endl;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_tracking");
    ros::NodeHandle nh;

    ros::Rate rate(25);

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

    // 获取 无人机ENU下的位置
    ros::Subscriber curr_pos_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(g_uav_id) + "/prometheus/state", 10, droneStateCb);
    // 获取视觉反馈
    ros::Subscriber vision_sub = nh.subscribe<spirecv_msgs::TargetsInFrame>("/uav" + std::to_string(g_uav_id) + "/spirecv/aruco_detection_with_d435i", 10, VisionCb);
    // 【发布】发送给prometheus_uav_control的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(g_uav_id) + "/prometheus/command", 10);
    // 获取遥控器状态
    ros::Subscriber uav_control_state_sub = nh.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(g_uav_id) + "/prometheus/control_state", 10, [&](const prometheus_msgs::UAVControlState::ConstPtr &msg) -> void
                                                                                      { g_uavcontrol_state = *msg; });
    // prometheus_msgs::UAVCommand comm;
    // comm.Command_ID = 0;
    // comm.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    // comm.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;

    // 到达那个二维码front
    // if (!checkInput(goto_id))
    //     return 0;

    while (ros::ok())
    {
        ros::spinOnce();
        if(g_uavcontrol_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, WHITE, "Waiting for enter COMMAND_CONTROL state");
            continue;
        }
        // if(g_Detection_raw.score != 1.0 || g_Detection_raw.mode == true)
        if(g_Detection_raw.mode == false)
        {
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            PCOUT(-1, GREEN, "Waiting for aruco detected!");
        }
        else
        {
            distance_to_setpoint = pos_body_frame.norm();
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
            // g_command_now.
            // 根据误差计算计算应该给于无人机的速度
            // g_command_now.velocity_ref[0] = kpx_track * (pos_body_enu_frame[0] - tracking_delta[0]);
            // g_command_now.velocity_ref[1] = kpy_track * (pos_body_enu_frame[1] - tracking_delta[1]);
            // g_command_now.velocity_ref[2] = kpz_track * (pos_body_enu_frame[2] - tracking_delta[2]);
            g_command_now.velocity_ref[0] = 0.5 * (g_Detection_raw.pz - 2.5);
            g_command_now.velocity_ref[1] = -0.8 * g_Detection_raw.px;
            g_command_now.velocity_ref[2] = -0.8 * g_Detection_raw.py;
            g_command_now.yaw_ref = 0;
            std::string tmp = "[aruco_tracking]: Tracking the Aruco, distance_to_setpoint : " + std::to_string(distance_to_setpoint) + " [m] " + "\n velocity_x: " + std::to_string(g_command_now.velocity_ref[0]) + "  [m/s], velocity_y: " + std::to_string(g_command_now.velocity_ref[1]) + " [m/s], velocity_y: " + std::to_string(g_command_now.velocity_ref[2]) + " [m/s]";
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
