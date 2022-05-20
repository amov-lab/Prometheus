// ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>

// topic 头文件
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include "message_utils.h"

using namespace std;
using namespace Eigen;
#define NODE_NAME "object_tracking"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::UAVState g_UAVState;
Eigen::Vector3f g_drone_pos;
//---------------------------------------Vision---------------------------------------------
prometheus_msgs::DetectionInfo g_Detection_raw; //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
Eigen::Vector3f pos_body_frame;
Eigen::Vector3f pos_body_enu_frame;    //原点位于质心，x轴指向前方，y轴指向左，z轴指向上的坐标系
float kpx_track, kpy_track, kpz_track; //控制参数 - 比例参数
float start_point_x, start_point_y, start_point_z, start_yaw;
bool is_detected = false;        // 是否检测到目标标志
int num_count_vision_lost = 0;   //视觉丢失计数器
int num_count_vision_regain = 0; //视觉丢失计数器
int g_uav_id;
int Thres_vision = 0;            //视觉丢失计数器阈值
Eigen::Vector3f camera_offset;
//---------------------------------------Track---------------------------------------------
float distance_to_setpoint;
Eigen::Vector3f tracking_delta;
//---------------------------------------Output---------------------------------------------
prometheus_msgs::UAVCommand g_command_now; //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();  //打印各项参数以供检查
void printf_result(); //打印函数
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

    nh.param<float>("start_point_x", start_point_x, 0.0);
    nh.param<float>("start_point_y", start_point_y, 0.0);
    nh.param<float>("start_point_z", start_point_z, 2.0);

    nh.param<int>("uav_id", g_uav_id, 1);

    // 【订阅】视觉消息 来自视觉节点
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    // 注意这里为了复用程序使用了/prometheus/object_detection/kcf_tracker作为话题名字，适用于椭圆、二维码、yolo等视觉算法
    // 故同时只能运行一种视觉识别程序，如果想同时追踪多个目标，这里请修改接口话题的名字
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/uav" + std::to_string(g_uav_id) + "/prometheus/object_detection/siamrpn_tracker", 10, vision_cb);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(g_uav_id) + "/prometheus/state", 10, drone_state_cb);

    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(g_uav_id) + "/prometheus/command", 10);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/uav" + std::to_string(g_uav_id) + "/prometheus/message/main", 10);
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    nh.param<float>("start_yaw", start_yaw, 0.0);

    //打印现实检查参数
    printf_param();
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    int start_flag = 0;
    while (start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Object Tracking Mission<<<<<<<<<<<<<<<<<<<<<< " << endl;
        cout << "Please check the parameter and setting，enter 1 to continue， else for quit: " << endl;
        cin >> start_flag;
    }

    // 起飞
    cout << "[object tracking]: "
         << "Takeoff to predefined position." << endl;
    g_command_now.Command_ID = 1;
    while (g_UAVState.position[2] < 0.3)
    {
        g_command_now.header.stamp = ros::Time::now();
        g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        g_command_now.Command_ID = g_command_now.Command_ID + 1;
        g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
        g_command_now.position_ref[0] = start_point_x;
        g_command_now.position_ref[1] = start_point_y;
        g_command_now.position_ref[2] = start_point_z;
        g_command_now.yaw_ref = start_yaw / 180 * 3.1415926;
        command_pub.publish(g_command_now);
        cout << "Takeoff ..." << endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    g_command_now.position_ref[0] = 0;
    g_command_now.position_ref[1] = 0;
    g_command_now.position_ref[2] = 0;

    // 先读取一些飞控的数据
    for (int i = 0; i < 10; i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::Duration(3.0).sleep();

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        g_command_now.header.stamp = ros::Time::now();
        g_command_now.Command_ID = g_command_now.Command_ID + 1;

        printf_result();

        // 对目标到直线距离
        distance_to_setpoint = pos_body_frame.norm();
        if (!is_detected)
        {
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            cout << "[object_tracking]: Lost the Target " << endl;
            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Lost the Target.");
        }
        else
        {
            cout << "[object_tracking]: Tracking the Target, distance_to_setpoint : " << distance_to_setpoint << " [m] " << endl;
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL; // xy velocity z position

            g_command_now.velocity_ref[0] = kpx_track * (pos_body_enu_frame[0] - tracking_delta[0]);
            g_command_now.velocity_ref[1] = kpy_track * (pos_body_enu_frame[1] - tracking_delta[1]);
            g_command_now.velocity_ref[2] = kpz_track * (tracking_delta[2] - g_UAVState.position[2] );
            g_command_now.yaw_ref = start_yaw / 180 * 3.1415926;
        }

        // Publish
        g_command_now.header.stamp = ros::Time::now();
        g_command_now.Command_ID = g_command_now.Command_ID + 1;
        command_pub.publish(g_command_now);

        rate.sleep();
    }

    return 0;
}

void printf_result()
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(4);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>Obeject Tracking<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    if (is_detected)
    {
        cout << "is_detected: ture" << endl;
    }
    else
    {
        cout << "is_detected: false" << endl;
    }

    cout << "Detection_raw: " << g_Detection_raw.position[0] << " [m] " << g_Detection_raw.position[1] << " [m] " << g_Detection_raw.position[2] << " [m] " << endl;
    cout << "Detection_raw: " << g_Detection_raw.attitude[2] / 3.1415926 * 180 << " [du] " << endl;

    cout << "pos_body_frame: " << pos_body_frame[0] << " [m] " << pos_body_frame[1] << " [m] " << pos_body_frame[2] << " [m] " << endl;
    cout << "pos_body_enu_frame: " << pos_body_enu_frame[0] << " [m] " << pos_body_enu_frame[1] << " [m] " << pos_body_enu_frame[2] << " [m] " << endl;

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "volicity_des: " << g_command_now.velocity_ref[0] << " [m] " << g_command_now.velocity_ref[1] << " [m] " << g_command_now.velocity_ref[2] << " [m] " << endl;
}

void printf_param()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "Thres_vision : " << Thres_vision << endl;
    cout << "uav id : " << g_uav_id << endl;

    cout << "tracking_delta_x : " << tracking_delta[0] << endl;
    cout << "tracking_delta_y : " << tracking_delta[1] << endl;
    cout << "tracking_delta_z : " << tracking_delta[2] << endl;

    cout << "kpx_track : " << kpx_track << endl;
    cout << "kpy_track : " << kpy_track << endl;
    cout << "kpz_track : " << kpz_track << endl;
    cout << "start_point_x : " << start_point_x << endl;
    cout << "start_point_y : " << start_point_y << endl;
    cout << "start_point_z : " << start_point_z << endl;
}
