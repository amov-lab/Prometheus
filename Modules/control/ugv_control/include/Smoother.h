/// @file Smoother.h
/// @author Vinson Sheep (775014077@qq.com)
/// @brief 
/// 
/// @version 2.0
/// @date 2021-04-28
/// 
/// @copyright Copyright (c) 2021
/// 

#ifndef _SMOOTHER_H_
#define _SMOOTHER_H_

#include "ros/ros.h"
// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/PointStamped.h"
#include <algorithm>	// min, max
#include <string>	// string
#include <memory> // shared_ptr

namespace TB2
{

using std::min;
using std::max;
using std::string;

class Smoother
{
private:

	double MAX_LIN_VEL_;		///< Maximum linear velocity
	double MAX_ANG_VEL_;		///< Maximum angular velocity
	double LIN_STEP_SIZE_;	///< Linear step size, in terms of linear accelerator
	double ANG_STEP_SIZE_; 	///< Angular step size, in terms of angular accelerator

	// double cur_linear_vel_;
	// double cur_angular_vel_;

	// ros::Publisher vel_pub_;

	/// @brief smooth input data according output and slot
	/// While output is much greater or lower than input, smoothing operation woule be token. Input
	/// data is being smoothed to get closer to ouput according to slot.
	/// @param input data being smoothing
	/// @param output  data to which input get close
	/// @param slot step size of smoothing
	/// @return const double smoothed data
	const double make_simple_profile(const double input, const double output, const double slot);
	
	/// @brief velocity constrain function
	/// Correct abnormal values.
	/// @param input data
	/// @param low minimum
	/// @param high maximum
	/// @return const double data between minimum and maximum
	const double constrain(const double input, const double low, const double high);

	

public:
	/// @brief Construct a new Smoother object
	/// @details 
	Smoother();

	/// @brief Destroy the Smoother object
	/// @details Do nothing.
	~Smoother();

	Smoother(const string name);


	// void pub(const double cur_linear_vel, 
	// 		const double cur_angular_vel, 
	// 		const double target_linear_vel, 
	// 		const double target_angular_vel)
	// {
	// 	cur_linear_vel_ = cur_linear_vel;
	// 	cur_angular_vel_ = cur_angular_vel;
	// 	pub(target_linear_vel, target_angular_vel);
	// }

	// void pub(const double target_linear_vel = 0.0, const double target_angular_vel = 0.0);

	void smooth(const double cur_linear_vel, const double cur_angular_vel,
			double &target_linear_vel, double &target_angular_vel);

	typedef std::shared_ptr<Smoother> Ptr; 
};


}


#endif