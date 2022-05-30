#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include "prometheus_msgs/UAVState.h"
#include "prometheus_msgs/UAVCommand.h"
#include "prometheus_msgs/UAVControlState.h"
#include "printf_utils.h"

#include "mission_utils.h"

using namespace std;
using namespace Eigen;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool g_use_pad_height; // 是否使用降落板绝对高度
float g_pad_height;
int g_uav_id;
string g_message;
std_msgs::Bool vision_switch;
float g_start_point[3]; // 起始降落位置
float g_camera_offset[3];
float g_target_vel_xy[2]; // 目标移动速度 enu坐标系 单位：m/s
std_msgs::Bool g_flag_start;
prometheus_msgs::UAVCommand g_command_now;
prometheus_msgs::UAVControlState g_uavcontrol_state; //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
#define NODE_NAME "autonomous_landing"
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::UAVState g_UAVState; // 无人机状态
Eigen::Matrix3f g_R_Body_to_ENU;      // 无人机机体系至惯性系转换矩阵
//---------------------------------------Vision---------------------------------------------
nav_msgs::Odometry g_GroundTruth; // 降落板真实位置（仿真中由Gazebo插件提供）
Detection_result g_landpad_det;   // 检测结果
//---------------------------------------Track---------------------------------------------
float g_kp_land[3]; //控制参数 - 比例参数

// 五种状态机
enum EXEC_STATE
{
    WAITING_RESULT,
    TRACKING,
    LOST,
    LANDING,
};
EXEC_STATE exec_state;

float g_distance_to_pad;
float g_arm_height_to_ground;
float g_arm_distance_to_pad;
//---------------------------------------Output---------------------------------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void landpadDetCb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    g_landpad_det.object_name = "landpad";
    g_landpad_det.Detection_info = *msg;
    // 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 相机安装误差 在mission_utils.h中设置场
    // x, y轴交换
    g_landpad_det.pos_body_frame[0] = -g_landpad_det.Detection_info.position[1] + g_camera_offset[0];
    g_landpad_det.pos_body_frame[1] = -g_landpad_det.Detection_info.position[0] + g_camera_offset[1];
    g_landpad_det.pos_body_frame[2] = -g_landpad_det.Detection_info.position[2] + g_camera_offset[2];

    // 机体系 -> 机体惯性系 (原点在机体的惯性系) (对无人机姿态进行解耦)
    g_landpad_det.pos_body_enu_frame = g_R_Body_to_ENU * g_landpad_det.pos_body_frame;

    if (g_use_pad_height)
    {
        //若已知降落板高度，则无需使用深度信息。
        g_landpad_det.pos_body_enu_frame[2] = g_pad_height - g_UAVState.position[2];
    }

    // 机体惯性系 -> 惯性系
    g_landpad_det.pos_enu_frame[0] = g_UAVState.position[0] + g_landpad_det.pos_body_enu_frame[0];
    g_landpad_det.pos_enu_frame[1] = g_UAVState.position[1] + g_landpad_det.pos_body_enu_frame[1];
    g_landpad_det.pos_enu_frame[2] = g_UAVState.position[2] + g_landpad_det.pos_body_enu_frame[2];
    // 此降落方案不考虑偏航角 （高级版可提供）
    g_landpad_det.att_enu_frame[2] = 0.0;

    if (g_landpad_det.Detection_info.detected)
    {
        g_landpad_det.num_regain++;
        g_landpad_det.num_lost = 0;
    }
    else
    {
        g_landpad_det.num_regain = 0;
        g_landpad_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if (g_landpad_det.num_lost > VISION_THRES)
    {
        g_landpad_det.is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if (g_landpad_det.num_regain > VISION_THRES)
    {
        g_landpad_det.is_detected = true;
    }
}

void droneStateCb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    g_UAVState = *msg;

    g_R_Body_to_ENU = get_rotation_matrix(g_UAVState.attitude[0], g_UAVState.attitude[1], g_UAVState.attitude[2]);
}

void groundtruthCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    g_GroundTruth = *msg;
}

void switchCb(const std_msgs::Bool::ConstPtr &msg)
{
    g_flag_start = *msg;
}

inline void readParams(const ros::NodeHandle &nh)
{
    nh.param<int>("uav_id", g_uav_id, 1);
    //强制上锁高度
    nh.param<float>("arm_height_to_ground", g_arm_height_to_ground, 0.4);
    //强制上锁距离
    nh.param<float>("arm_distance_to_pad", g_arm_distance_to_pad, 0.3);
    // 是否使用降落板绝对高度
    nh.param<bool>("use_pad_height", g_use_pad_height, false);
    nh.param<float>("pad_height", g_pad_height, 0.01);

    //追踪控制参数
    nh.param<float>("kpx_land", g_kp_land[0], 0.1);
    nh.param<float>("kpy_land", g_kp_land[1], 0.1);
    nh.param<float>("kpz_land", g_kp_land[2], 0.1);

    // 初始起飞点
    nh.param<float>("start_point_x", g_start_point[0], 0.0);
    nh.param<float>("start_point_y", g_start_point[1], 0.0);
    nh.param<float>("start_point_z", g_start_point[2], 1.0);

    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置
    nh.param<float>("camera_offset_x", g_camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", g_camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", g_camera_offset[2], 0.0);
}

inline void topicSub(ros::NodeHandle &nh)
{
    //【订阅】降落板与无人机的相对位置及相对偏航角  单位：米   单位：弧度
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    static ros::Subscriber landpad_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/landpad_det", 10, landpadDetCb);

    //【订阅】无人机状态
    static ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(g_uav_id) + "/prometheus/state", 10, droneStateCb);

    //【订阅】地面真值，此信息仅做比较使用 不强制要求提供
    static ros::Subscriber groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/landing_pad", 10, groundtruthCb);

    //【订阅】降落程序开关，默认情况下不启用，用于多任务情况
    static ros::Subscriber switch_sub = nh.subscribe<std_msgs::Bool>("/prometheus/switch/landing", 10, switchCb);
    // 订阅遥控器状态
    static ros::Subscriber uav_control_state_sub = nh.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(g_uav_id) + "/prometheus/control_state", 10, [&](const prometheus_msgs::UAVControlState::ConstPtr &msg) -> void
                                                                                                  { g_uavcontrol_state = *msg; });
}

static ros::Publisher g_vision_switch_pub, g_command_pub, g_message_pub;

inline void topicAdv(ros::NodeHandle &nh)
{
    // 【发布】 视觉模块开关量
    g_vision_switch_pub = nh.advertise<std_msgs::Bool>("/uav" + std::to_string(g_uav_id) + "/prometheus/switch/landpad_det", 10);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    g_command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(g_uav_id) + "/prometheus/command", 10);

    // 【发布】用于地面站显示的提示消息
    g_message_pub = nh.advertise<prometheus_msgs::Message>("/uav" + std::to_string(g_uav_id) + "/prometheus/message/main", 10);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_landing");
    ros::NodeHandle nh("~");

    // 节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(30.0);

    // 读取配置参数
    readParams(nh);
    // 订阅话题
    topicSub(nh);
    // 发布话题
    topicAdv(nh);

    g_command_now.Command_ID = 1;
    exec_state = EXEC_STATE::WAITING_RESULT;
    while (ros::ok())
    {
        //回调
        ros::spinOnce();
        // 等待进入COMMAND_CONTROL模式
        while (g_uavcontrol_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, TAIL, "Waiting for enter COMMAND_CONTROL state");
            continue;
        }

        switch (exec_state)
        {
        // 初始状态，等待视觉检测结果
        case WAITING_RESULT:
        {
            if (g_landpad_det.is_detected)
            {
                exec_state = TRACKING;
                break;
            }

            // 发送视觉节点启动指令
            vision_switch.data = true;
            g_vision_switch_pub.publish(vision_switch);
            PCOUT(-1, GREEN, "Waiting for the detection result.");
            break;
        }
        // 追踪状态
        case TRACKING:
        {
            // 正常追踪
            char message_chars[256];
            sprintf(message_chars, "Tracking the Landing Pad, distance_to_the_pad :   %f [m] .", g_distance_to_pad);
            g_message = message_chars;
            cout << g_message << endl;
            // 每1秒打印一次到pad的距离
            PCOUT(1, GREEN, g_message);

            // 丢失,进入LOST状态
            if (!g_landpad_det.is_detected)
            {
                exec_state = LOST;
                PCOUT(1, YELLOW, "Lost the Landing Pad.");
                break;
            }

            // 抵达上锁点,进入LANDING
            g_distance_to_pad = g_landpad_det.pos_body_enu_frame.norm();
            //　达到降落距离，上锁降落
            if (g_distance_to_pad < g_arm_distance_to_pad)
            {
                exec_state = LANDING;
                PCOUT(0, GREEN, "Catched the Landing Pad.");
                break;
            }
            //　达到最低高度，上锁降落
            else if (abs(g_landpad_det.pos_body_enu_frame[2]) < g_arm_height_to_ground)
            {
                exec_state = LANDING;
                PCOUT(0, GREEN, "Reach the lowest height.");
                break;
            }

            // 机体系速度控制
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL; // xy velocity z position

            for (int i = 0; i < 3; i++)
            {
                g_command_now.velocity_ref[i] = g_kp_land[i] * g_landpad_det.pos_body_enu_frame[i];
            }

            g_command_now.yaw_ref = 0.0;

            break;
        }
        case LOST:
        {
            static int lost_time = 0;
            lost_time++;

            // 重新获得信息,进入TRACKING
            if (g_landpad_det.is_detected)
            {
                exec_state = TRACKING;
                PCOUT(0, GREEN, "Regain the Landing Pad.");
                break;
            }

            // 首先是悬停等待 尝试得到图像, 如果仍然获得不到图像 则原地上升
            if (lost_time < 10.0)
            {
                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;

                ros::Duration(0.4).sleep();
            }
            else
            {
                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL;
                g_command_now.velocity_ref[0] = 0.0;
                g_command_now.velocity_ref[1] = 0.0;
                g_command_now.velocity_ref[2] = 0.1;
                g_command_now.yaw_ref = 0;

                // 如果上升超过原始高度，则认为任务失败，则直接降落
                if (g_UAVState.position[2] >= g_start_point[2])
                {
                    exec_state = LANDING;
                    lost_time = 0;
                    PCOUT(0, RED, "Mission failed, landing... ");
                }
            }
            break;
        }
        case LANDING:
        {
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Land;
            break;
        }
        }
        g_command_now.header.stamp = ros::Time::now();
        g_command_now.Command_ID = g_command_now.Command_ID + 1;
        g_command_pub.publish(g_command_now);
        rate.sleep();
    }

    return 0;
}