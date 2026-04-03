/// @file Smoother.cpp
/// @author Vinson Sheep (775014077@qq.com)
/// @brief 
/// @version 2.0
/// @date 2021-04-28
/// 
/// @copyright Copyright (c) 2021
/// 

#include "Smoother.h"

namespace TB2
{

Smoother::Smoother(){}
Smoother::~Smoother(){}

Smoother::Smoother(const string name)
{
    ros::NodeHandle private_nh("~" + name);

    private_nh.param("max_lin_vel", MAX_LIN_VEL_, 1.00);
    private_nh.param("max_ang_vel", MAX_ANG_VEL_, 2.00);
    private_nh.param("lin_step_size", LIN_STEP_SIZE_, 0.10);
    private_nh.param("ang_step_size", ANG_STEP_SIZE_, 0.20); 

    ROS_INFO("%f, %f, %f, %f", MAX_LIN_VEL_, MAX_ANG_VEL_, LIN_STEP_SIZE_, ANG_STEP_SIZE_);
}

const double Smoother::make_simple_profile(const double input, const double output,
                                         const double slot){
    if (input > output){
        return min(input, output + slot/2);
    }
    else if (input < output){
        return max(input, output - slot/2);
    }
    return output;
}

//设置input的阈值为low～high
const double Smoother::constrain(const double input, const double low, const double high){
    if (input > high)
        return high;
    else if (input < low)
        return low;
    return input;
}


void Smoother::smooth(const double cur_linear_vel, const double cur_angular_vel,
			double &target_linear_vel, double &target_angular_vel){
    target_linear_vel = constrain(target_linear_vel, -MAX_LIN_VEL_, MAX_LIN_VEL_);
    target_linear_vel = make_simple_profile(target_linear_vel, 
                        cur_linear_vel, LIN_STEP_SIZE_);
    target_angular_vel = constrain(target_angular_vel, -MAX_ANG_VEL_, MAX_ANG_VEL_);
    target_angular_vel = make_simple_profile(target_angular_vel, 
                        cur_angular_vel, ANG_STEP_SIZE_);
    
}

}
