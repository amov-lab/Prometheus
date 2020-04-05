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

using namespace std;

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
