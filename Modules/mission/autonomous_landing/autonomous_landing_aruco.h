#ifndef AUTONOMOUS_LANDING_ARUCO_H
#define AUTONOMOUS_LANDING_ARUCO_H

//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>

#include <prometheus_msgs/ArucoInfo.h>

#include "mission_utils.h"
#include "message_utils.h"

using namespace std;
using namespace Eigen;

#define NODE_NAME "autonomous_landing_aruco"

// 参数
bool hold_mode; // 悬停模式，用于测试检测精度
bool sim_mode;  // 选择Gazebo仿真模式 或 真实实验模式
float camera_offset[3];     //相机安装偏差

string message;

prometheus_msgs::DroneState _DroneState;    // 无人机状态
Eigen::Vector3f mav_pos;
Eigen::Matrix3f R_Body_to_ENU;              // 无人机机体系至惯性系转换矩阵
Eigen::Vector3f return_point;

nav_msgs::Odometry GroundTruth;             // 降落板真实位置（仿真中由Gazebo插件提供）
prometheus_msgs::ArucoInfo marker_info;               // 检测结果
Eigen::Vector3f marker_body,marker_body_enu;
Eigen::Vector3f marker_enu;
bool is_detected;
int num_detected, num_lost;
float distance_to_pad;

prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令

Eigen::Vector3f search_point;
float dis_to_search_point;
int search_id;
bool start_search;

float kp_land[3];         //控制参数 - 比例参数

// 状态机
enum EXEC_STATE
{
    INIT,
    TAKEOFF,
    SEARCH,
    HORIZON_APPROACH,
    DECEND,
    LAND_NOW,
    DISARM,
};
EXEC_STATE exec_state;

ros::Subscriber aruco_sub, drone_state_sub, groundtruth_sub;
ros::Publisher command_pub, message_pub;

void mainloop_cb(const ros::TimerEvent& e);
void search_cb(const ros::TimerEvent& e);
void control_cb(const ros::TimerEvent& e);
void printf_cb(const ros::TimerEvent& e);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void aruco_cb(const prometheus_msgs::ArucoInfo::ConstPtr &msg)
{
    marker_info = *msg;

    if(marker_info.detected)
    {
        num_detected++;
        num_lost = 0;
    }else
    {
        num_detected = 0;
        num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(num_lost > VISION_THRES)
    {
        is_detected = false;
        return;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(num_detected > VISION_THRES)
    {
        is_detected = true;
    }

    // marker在无人机机体系下的位置
    marker_body[0] = - marker_info.position[1] + camera_offset[0];
    marker_body[1] = - marker_info.position[0] + camera_offset[1];
    marker_body[2] = - marker_info.position[2] + camera_offset[2];

    distance_to_pad = marker_body.norm();

    // 机体系 -> 机体惯性系 (原点在机体的惯性系) (对无人机姿态进行解耦)
    marker_body_enu = R_Body_to_ENU * marker_body;

    // 机体惯性系 -> 惯性系
    marker_enu[0] = _DroneState.position[0] + marker_body_enu[0];
    marker_enu[1] = _DroneState.position[1] + marker_body_enu[1];
    marker_enu[2] = _DroneState.position[2] + marker_body_enu[2];
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    mav_pos << _DroneState.position[0], _DroneState.position[1], _DroneState.position[2];

    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);
}

void groundtruth_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    GroundTruth = *msg;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "hold_mode : "<< hold_mode << endl;
    cout << "sim_mode : "<< sim_mode << endl;

    cout << "kpx_land : "<< kp_land[0] << endl;
    cout << "kpy_land : "<< kp_land[1] << endl;
    cout << "kpz_land : "<< kp_land[2] << endl;
    cout << "camera_offset_x : "<< camera_offset[0] << endl;
    cout << "camera_offset_y : "<< camera_offset[1] << endl;
    cout << "camera_offset_z : "<< camera_offset[2] << endl;
}

#endif 