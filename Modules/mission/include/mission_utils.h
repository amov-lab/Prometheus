/***************************************************************************************************************************
* mission_utils.h
*
* Author: Qyp
*
* Update Time: 2020.1.12
*
* Introduction:  mission_utils
*
***************************************************************************************************************************/

#ifndef MISSION_UTILS_H
#define MISSION_UTILS_H

#include <Eigen/Eigen>
#include <math.h>

//topic 头文件
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/MultiDetectionInfo.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/Message.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

using namespace std;

#define DIS_THRES 0.1
#define VISION_THRES 10

//相机安装OFFSET
#define FRONT_CAMERA_OFFSET_X 0.2
#define FRONT_CAMERA_OFFSET_Y 0.0
#define FRONT_CAMERA_OFFSET_Z -0.05

#define DOWN_CAMERA_OFFSET_X 0.0
#define DOWN_CAMERA_OFFSET_Y 0.0
#define DOWN_CAMERA_OFFSET_Z -0.1


// 定义视觉检测结构体
struct Detection_result
{
    string object_name;
    // 视觉检测原始信息，返回的结果为相机坐标系
    // 方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    prometheus_msgs::DetectionInfo Detection_info;      
    // 目标在机体系位置
    Eigen::Vector3f pos_body_frame;   
    // 目标在机体-惯性系位置 (原点位于质心，x轴指向前方，y轴指向左，z轴指向上的坐标系)
    Eigen::Vector3f pos_body_enu_frame;  
    // 目标在惯性系位置 (原点位于起飞点，x轴指向前方，y轴指向左，z轴指向上的坐标系)
    Eigen::Vector3f pos_enu_frame; 
    // 目标在机体系姿态
    Eigen::Vector3f att_body_frame;
    // 目标在惯性系姿态
    Eigen::Vector3f att_enu_frame;
    // 目标识别标志位,阈值:VISION_THRES
    bool is_detected = false; 
    int num_lost = 0;          //视觉丢失计数器
    int num_regain = 0;     
};


//打印视觉检测消息
void printf_detection_result(const struct Detection_result& det_info)
{
    cout << "Object_name    : " << det_info.object_name <<  endl;
    if(det_info.is_detected)
    {
    cout << "is_detected    : " << "True" <<  endl;
    }else
    {
    cout << "is_detected    : " << "False" <<  endl;
    }
    cout << "Camera_frame   : " << det_info.Detection_info.position[0] << " [m] "<< det_info.Detection_info.position[1] << " [m] "<< det_info.Detection_info.position[2] << " [m] "<<endl;
    cout << "Body_frame     : " << det_info.pos_body_frame[0] << " [m] "<< det_info.pos_body_frame[1] << " [m] "<< det_info.pos_body_frame[2] << " [m] "<<endl;
    cout << "BodyENU_frame  : " << det_info.pos_body_enu_frame[0] << " [m] "<< det_info.pos_body_enu_frame[1] << " [m] "<< det_info.pos_body_enu_frame[2] << " [m] "<<endl;
    cout << "ENU_frame      : " << det_info.pos_enu_frame[0] << " [m] "<< det_info.pos_enu_frame[1] << " [m] "<< det_info.pos_enu_frame[2] << " [m] "<<endl;

}
    


float cal_distance(const Eigen::Vector3f& pos_drone,const Eigen::Vector3f& pos_target)
{
    Eigen::Vector3f relative;
    relative =  pos_target - pos_drone; 
    return relative.norm(); 
}

float cal_distance_tracking(const Eigen::Vector3f& pos_drone,const Eigen::Vector3f& pos_target,const Eigen::Vector3f& delta)
{
    Eigen::Vector3f relative;
    relative =  pos_target - pos_drone - delta; 
    return relative.norm(); 
}

//constrain_function
float constrain_function(float data, float Max)
{
    if(abs(data)>Max)
    {
        return (data > 0) ? Max : -Max;
    }
    else
    {
        return data;
    }
}

//constrain_function2
float constrain_function2(float data, float Min,float Max)
{
    if(data > Max)
    {
        return Max;
    }
    else if (data < Min)
    {
        return Min;
    }else
    {
        return data;
    }
}

//sign_function
float sign_function(float data)
{
    if(data>0)
    {
        return 1.0;
    }
    else if(data<0)
    {
        return -1.0;
    }
    else if(data == 0)
    {
        return 0.0;
    }
}

// min function
float min(float data1,float data2)
{
    if(data1>=data2)
    {
        return data2;
    }
    else
    {
        return data1;
    }
}

//旋转矩阵：机体系到惯性系
Eigen::Matrix3f get_rotation_matrix(float phi, float theta, float psi)
{
    Eigen::Matrix3f R_Body_to_ENU;

    float r11 = cos(theta)*cos(psi);
    float r12 = - cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi);
    float r13 = sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi);
    float r21 = cos(theta)*sin(psi);
    float r22 = cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi);
    float r23 = - sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi);
    float r31 = - sin(theta);
    float r32 = sin(phi)*cos(theta);
    float r33 = cos(phi)*cos(theta); 
    R_Body_to_ENU << r11,r12,r13,r21,r22,r23,r31,r32,r33;

    return R_Body_to_ENU;
}
#endif
