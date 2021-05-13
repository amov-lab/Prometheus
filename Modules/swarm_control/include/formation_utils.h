#ifndef FORMATION_UTILS_H
#define FORMATION_UTILS_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include <prometheus_msgs/Message.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>
#include <prometheus_msgs/LogMessage.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

using namespace Eigen;

float one_column_shape_for_4uav[16] = {
    0.5,0.0,0.0,0.0,    -0.5,0.0,0.0,0.0,   1.5,0.0,0.0,0.0,    -1.5,0.0,0.0,0.0};

float triangle_shape_for_4uav[16] = {
    0.5,0.5,0.0,0.0,   -0.5,0.5,0.0,0.0,   1.5,-0.5,0.0,0.0,    -1.5,-0.5,0.0,0.0};

float square_shape_for_4uav[16] = {
    1.0,1.0,0.0,0.0,   -1.0,1.0,0.0,0.0,   1.0,-1.0,0.0,0.0,    -1.0,-1.0,0.0,0.0};

float circular_shape_for_4uav[16] = {
    0.0,2.0,0.0,0.0,   0.0,-2.0,0.0,0.0,   2.0,0.0,0.0,0.0,    -2.0,0.0,0.0,0.0};

float one_column_shape_for_8uav[32] = {
    0.5,0.0,0.0,0.0,    -0.5,0.0,0.0,0.0,   1.5,0.0,0.0,0.0,    -1.5,0.0,0.0,0.0,
    2.5,0.0,0.0,0.0,    -2.5,0.0,0.0,0.0,   3.5,0.0,0.0,0.0,    -3.5,0.0,0.0,0.0};

float triangle_shape_for_8uav[32] = {
    0.5,2.0,0.0,0.0,    -0.5,2.0,0.0,0.0,   1.5,1.0,0.0,0.0,    -1.5,1.0,0.0,0.0,
    2.5,0.0,0.0,0.0,    -2.5,0.0,0.0,0.0,   3.5,-1.0,0.0,0.0,   -3.5,-1.0,0.0,0.0};

float square_shape_for_8uav[32] = {
    0.5,-2.0,0.0,0.0,   -0.5,2.0,0.0,0.0,   2.5,2.0,0.0,0.0,    -2.5,2.0,0.0,0.0,
    2.5,0.0,0.0,0.0,    -2.5,0.0,0.0,0.0,   2.5,-2.0,0.0,0.0,   -2.5,-2.0,0.0,0.0};

float circular_shape_for_8uav[32] = {
    0.5,-1.0,0.0,0.0,   -0.5,1.0,0.0,0.0,   2.0,1.0,0.0,0.0,    -2.0,1.0,0.0,0.0,
    2.0,-1.0,0.0,0.0,   -2.0,-1.0,0.0,0.0,  3.5,0.0,0.0,0.0,    -3.5,0.0,0.0,0.0};

float one_column_shape_for_40uav[160] = {
    0.5,0.0,0.0,0.0,    -0.5,0.0,0.0,0.0,   1.5,0.0,0.0,0.0,    -1.5,0.0,0.0,0.0,
    2.5,0.0,0.0,0.0,    -2.5,0.0,0.0,0.0,   3.5,0.0,0.0,0.0,    -3.5,0.0,0.0,0.0,
    4.5,0.0,0.0,0.0,    -4.5,0.0,0.0,0.0,   5.5,0.0,0.0,0.0,    -5.5,0.0,0.0,0.0,
    6.5,0.0,0.0,0.0,    -6.5,0.0,0.0,0.0,   7.5,0.0,0.0,0.0,    -7.5,0.0,0.0,0.0,
    8.5,0.0,0.0,0.0,    -8.5,0.0,0.0,0.0,   9.5,0.0,0.0,0.0,    -9.5,0.0,0.0,0.0,
    10.5,0.0,0.0,0.0,   -10.5,0.0,0.0,0.0,  11.5,0.0,0.0,0.0,   -11.5,0.0,0.0,0.0,
    12.5,0.0,0.0,0.0,   -12.5,0.0,0.0,0.0,  13.5,0.0,0.0,0.0,   -13.5,0.0,0.0,0.0,
    14.5,0.0,0.0,0.0,   -14.5,0.0,0.0,0.0,  15.5,0.0,0.0,0.0,   -15.5,0.0,0.0,0.0,
    16.5,0.0,0.0,0.0,   -16.5,0.0,0.0,0.0,  17.5,0.0,0.0,0.0,   -17.5,0.0,0.0,0.0,
    18.5,0.0,0.0,0.0,   -18.5,0.0,0.0,0.0,  19.5,0.0,0.0,0.0,   -19.5,0.0,0.0,0.0};

float triangle_shape_for_40uav[160] = {
    0.5,10.0,0.0,0.0,   -0.5,10.0,0.0,0.0,  1.5,9.0,0.0,0.0,    -1.5,9.0,0.0,0.0,
    2.5,8.0,0.0,0.0,    -2.5,8.0,0.0,0.0,   3.5,7.0,0.0,0.0,    -3.5,7.0,0.0,0.0,
    4.5,6.0,0.0,0.0,    -4.5,6.0,0.0,0.0,   5.5,5.0,0.0,0.0,    -5.5,5.0,0.0,0.0,
    6.5,4.0,0.0,0.0,    -6.5,4.0,0.0,0.0,   7.5,3.0,0.0,0.0,    -7.5,3.0,0.0,0.0,
    8.5,2.0,0.0,0.0,    -8.5,2.0,0.0,0.0,   9.5,1.0,0.0,0.0,    -9.5,1.0,0.0,0.0,
    10.5,0.0,0.0,0.0,   -10.5,0.0,0.0,0.0,  11.5,-1.0,0.0,0.0,  -11.5,-1.0,0.0,0.0,
    12.5,-2.0,0.0,0.0,  -12.5,-2.0,0.0,0.0, 13.5,-3.0,0.0,0.0,  -13.5,-3.0,0.0,0.0,
    14.5,-4.0,0.0,0.0,  -14.5,-4.0,0.0,0.0, 15.5,-5.0,0.0,0.0,  -15.5,-5.0,0.0,0.0,
    16.5,-6.0,0.0,0.0,  -16.5,-6.0,0.0,0.0, 17.5,-7.0,0.0,0.0,  -17.5,-7.0,0.0,0.0,
    18.5,-8.0,0.0,0.0,  -18.5,-8.0,0.0,0.0, 19.5,-9.0,0.0,0.0,  -19.5,-9.0,0.0,0.0};

float square_shape_for_40uav[160] = {
    0.5,0.0,0.0,0.0,    -0.5,0.0,0.0,0.0,   1.5,0.0,0.0,0.0,    -1.5,0.0,0.0,0.0,
    2.5,0.0,0.0,0.0,    -2.5,0.0,0.0,0.0,   3.5,0.0,0.0,0.0,    -3.5,0.0,0.0,0.0,
    4.5,0.0,0.0,0.0,    -4.5,0.0,0.0,0.0,   5.5,0.0,0.0,0.0,    -5.5,0.0,0.0,0.0,
    6.5,0.0,0.0,0.0,    -6.5,0.0,0.0,0.0,   7.5,0.0,0.0,0.0,    -7.5,0.0,0.0,0.0,
    8.5,0.0,0.0,0.0,    -8.5,0.0,0.0,0.0,   9.5,0.0,0.0,0.0,    -9.5,0.0,0.0,0.0,
    10.5,0.0,0.0,0.0,   -10.5,0.0,0.0,0.0,  11.5,0.0,0.0,0.0,   -11.5,0.0,0.0,0.0,
    12.5,0.0,0.0,0.0,   -12.5,0.0,0.0,0.0,  13.5,0.0,0.0,0.0,   -13.5,0.0,0.0,0.0,
    14.5,0.0,0.0,0.0,   -14.5,0.0,0.0,0.0,  15.5,0.0,0.0,0.0,   -15.5,0.0,0.0,0.0,
    16.5,0.0,0.0,0.0,   -16.5,0.0,0.0,0.0,  17.5,0.0,0.0,0.0,   -17.5,0.0,0.0,0.0,
    18.5,0.0,0.0,0.0,   -18.5,0.0,0.0,0.0,  19.5,0.0,0.0,0.0,   -19.5,0.0,0.0,0.0};

float circular_shape_for_40uav[160] = {
    0.5,0.0,0.0,0.0,    -0.5,0.0,0.0,0.0,   1.5,0.0,0.0,0.0,    -1.5,0.0,0.0,0.0,
    2.5,0.0,0.0,0.0,    -2.5,0.0,0.0,0.0,   3.5,0.0,0.0,0.0,    -3.5,0.0,0.0,0.0,
    4.5,0.0,0.0,0.0,    -4.5,0.0,0.0,0.0,   5.5,0.0,0.0,0.0,    -5.5,0.0,0.0,0.0,
    6.5,0.0,0.0,0.0,    -6.5,0.0,0.0,0.0,   7.5,0.0,0.0,0.0,    -7.5,0.0,0.0,0.0,
    8.5,0.0,0.0,0.0,    -8.5,0.0,0.0,0.0,   9.5,0.0,0.0,0.0,    -9.5,0.0,0.0,0.0,
    10.5,0.0,0.0,0.0,   -10.5,0.0,0.0,0.0,  11.5,0.0,0.0,0.0,   -11.5,0.0,0.0,0.0,
    12.5,0.0,0.0,0.0,   -12.5,0.0,0.0,0.0,  13.5,0.0,0.0,0.0,   -13.5,0.0,0.0,0.0,
    14.5,0.0,0.0,0.0,   -14.5,0.0,0.0,0.0,  15.5,0.0,0.0,0.0,   -15.5,0.0,0.0,0.0,
    16.5,0.0,0.0,0.0,   -16.5,0.0,0.0,0.0,  17.5,0.0,0.0,0.0,   -17.5,0.0,0.0,0.0,
    18.5,0.0,0.0,0.0,   -18.5,0.0,0.0,0.0,  19.5,0.0,0.0,0.0,   -19.5,0.0,0.0,0.0};

namespace formation_utils 
{
// 输入参数：　阵型，阵型基本尺寸，集群数量
// 所有的阵型和数量必须提前预设!!
Eigen::MatrixXf get_formation_separation(int swarm_shape, float swarm_size, int swarm_num)
{
    //矩阵大小为　swarm_num＊4 , 对应x,y,z,yaw四个自由度的分离量
    Eigen::MatrixXf seperation(swarm_num,4); 
    // cxy 默认swarm_size为１米
    // 横向一字型，虚拟领机位置为中心位置，其余飞机根据数量向左右增加
    if(swarm_shape == prometheus_msgs::SwarmCommand::One_column)
    {
        if(swarm_num == 8)
        {
            seperation = Map<Matrix<float,8,4,RowMajor>>(one_column_shape_for_8uav); 
        }
        else if(swarm_num == 40)
        {
            seperation = Map<Matrix<float,40,4,RowMajor>>(one_column_shape_for_40uav); 
        }else if(swarm_num == 4)
        {
            seperation = Map<Matrix<float,4,4,RowMajor>>(one_column_shape_for_4uav); 
        }
    }

    // 三角型，虚拟领机位置为中心位置
    if(swarm_shape == prometheus_msgs::SwarmCommand::Triangle)
    {
        if(swarm_num == 8)
        {
            seperation = Map<Matrix<float,8,4,RowMajor>>(triangle_shape_for_8uav); 
        }
        else if(swarm_num == 40)
        {
            seperation = Map<Matrix<float,40,4,RowMajor>>(triangle_shape_for_40uav); 
        }else if(swarm_num == 4)
        {
            seperation = Map<Matrix<float,4,4,RowMajor>>(triangle_shape_for_4uav); 
        }
    }

    // 方型，虚拟领机位置为中心位置
    if(swarm_shape == prometheus_msgs::SwarmCommand::Square)
    {
        if(swarm_num == 8)
        {
            seperation = Map<Matrix<float,8,4,RowMajor>>(square_shape_for_8uav); 
        }
        else if(swarm_num == 40)
        {
            seperation = Map<Matrix<float,40,4,RowMajor>>(square_shape_for_40uav); 
        }else if(swarm_num == 4)
        {
            seperation = Map<Matrix<float,4,4,RowMajor>>(square_shape_for_4uav); 
        }
    }

    // 圆形，虚拟领机位置为中心位置
    if(swarm_shape == prometheus_msgs::SwarmCommand::Circular)
    {
        if(swarm_num == 8)
        {
            seperation = Map<Matrix<float,8,4,RowMajor>>(circular_shape_for_8uav); 
        }
        else if(swarm_num == 40)
        {
            seperation = Map<Matrix<float,40,4,RowMajor>>(circular_shape_for_40uav); 
        }else if(swarm_num == 4)
        {
            seperation = Map<Matrix<float,4,4,RowMajor>>(circular_shape_for_4uav); 
        }
    }

    for(int i = 0 ; i < swarm_num ; i++)
    {
        for(int j = 0 ; j < 4; j++)
        {
            seperation(i,j) *= swarm_size;
        }
    }

    return seperation;
}
}
#endif