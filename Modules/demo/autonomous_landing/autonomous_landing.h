#ifndef AUTONOMOUS_LANDING_H
#define AUTONOMOUS_LANDING_H

//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>

#include <prometheus_msgs/ArucoInfo.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>

#include "math_utils.h"
#include "printf_utils.h"

using namespace std;

#define NODE_NAME "autonomous_landing"          // 节点名字

struct Parameter
{
    bool hold_mode;                                 //【参数】悬停模式，用于测试检测精度
    bool sim_mode;                                  //【参数】选择Gazebo仿真模式 或 真实实验模式
    float camera_offset[3];                         //【参数】相机安装偏差
    float kp_land[3];                               //【参数】控制比例参数
};
Parameter param;

enum EXEC_STATE
{
    INIT,
    SEARCH,
    HORIZON_APPROACH,
    DECEND,
    LAND_NOW
};
EXEC_STATE exec_state; // 状态机

ros::Subscriber aruco_sub;
ros::Subscriber groundtruth_sub;
ros::Subscriber uav_state_sub;
ros::Publisher  uav_command_pub;

prometheus_msgs::UAVState uav_state;                 // 无人机状态
prometheus_msgs::UAVCommand uav_command;             // 指令


struct VISION_INFO
{
    bool is_detected;
    int num_detected;
    int num_lost;
    float distance_to_pad;
    prometheus_msgs::ArucoInfo aruco_info;              // 检测结果
    Eigen::Vector3d aruco_body;
    Eigen::Vector3d aruco_body_enu;
    Eigen::Vector3d aruco_enu;
    nav_msgs::Odometry GroundTruth;           // 降落板真实位置（仿真中由Gazebo插件提供）  
};
VISION_INFO vision_info;

Eigen::Vector3d uav_pos;
Eigen::Matrix3d R_Body_to_ENU;              // 无人机机体系至惯性系转换矩阵
Eigen::Vector3d return_point;


Eigen::Vector3d search_point;
float dis_to_search_point;
int search_id;
bool start_search;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void aruco_cb(const prometheus_msgs::ArucoInfo::ConstPtr &msg)
{
    vision_info.aruco_info = *msg;

    if(msg->detected)
    {
        vision_info.num_detected++;
        vision_info.num_lost = 0;
    }else
    {
        vision_info.num_detected = 0;
        vision_info.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(vision_info.num_lost > VISION_THRES)
    {
        vision_info.is_detected = false;
        return;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(vision_info.num_detected > VISION_THRES)
    {
        vision_info.is_detected = true;
    }

    // aruco在无人机机体系下的位置
    vision_info.aruco_body[0] = - vision_info.aruco_info.position[1] + param.camera_offset[0];
    vision_info.aruco_body[1] = - vision_info.aruco_info.position[0] + param.camera_offset[1];
    vision_info.aruco_body[2] = - vision_info.aruco_info.position[2] + param.camera_offset[2];

    vision_info.distance_to_pad = vision_info.aruco_body.norm();

    // 机体系 -> 机体惯性系 (原点在机体的惯性系) (对无人机姿态进行解耦)
    vision_info.aruco_body_enu = R_Body_to_ENU * vision_info.aruco_body;

    // 机体惯性系 -> 惯性系
    vision_info.aruco_enu = uav_pos + vision_info.aruco_body_enu;
}

void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr& msg)
{
    uav_state = *msg;
    uav_pos = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    R_Body_to_ENU = get_rotation_matrix(msg->attitude[0], msg->attitude[1], msg->attitude[2]);
}

void groundtruth_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    vision_info.GroundTruth = *msg;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "hold_mode : "<< param.hold_mode << endl;
    cout << "sim_mode : "<< param.sim_mode << endl;

    cout << "kpx_land : "<< param.kp_land[0] << endl;
    cout << "kpy_land : "<< param.kp_land[1] << endl;
    cout << "kpz_land : "<< param.kp_land[2] << endl;
    cout << "camera_offset_x : "<< param.camera_offset[0] << endl;
    cout << "camera_offset_y : "<< param.camera_offset[1] << endl;
    cout << "camera_offset_z : "<< param.camera_offset[2] << endl;
}

#endif 