#ifndef CONTROLLER_TEST_H
#define CONTROLLER_TEST_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include <prometheus_msgs/UAVCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> 
#include "printf_utils.h"

using namespace std;

class Controller_Test
{
    public:
        //构造函数
        Controller_Test(void):
            Controller_Test_nh("~")
        {
            Controller_Test_nh.param<float>("Controller_Test/Circle/Center_x", circle_center[0], 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Circle/Center_y", circle_center[1], 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Circle/Center_z", circle_center[2], 1.0);
            Controller_Test_nh.param<float>("Controller_Test/Circle/circle_radius", circle_radius, 2.0);
            Controller_Test_nh.param<float>("Controller_Test/Circle/linear_vel", linear_vel, 0.5);
            Controller_Test_nh.param<float>("Controller_Test/Circle/direction", direction, 1.0);

            Controller_Test_nh.param<float>("Controller_Test/Eight/Center_x", eight_origin_[0], 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Eight/Center_y", eight_origin_[1], 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Eight/Center_z", eight_origin_[2], 1.0);
            Controller_Test_nh.param<float>("Controller_Test/Eight/omega", eight_omega_, 0.5);
            Controller_Test_nh.param<float>("Controller_Test/Eight/radial", radial, 2.0);

            Controller_Test_nh.param<float>("Controller_Test/Step/step_length", step_length, 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Step/step_interval", step_interval, 0.0);

        }

        //Printf the Controller_Test parameter
        void printf_param();

        //Controller_Test Calculation [Input: time_from_start; Output: Trajectory;]
        prometheus_msgs::UAVCommand Circle_trajectory_generation(float time_from_start);

        prometheus_msgs::UAVCommand Eight_trajectory_generation(float time_from_start);

        prometheus_msgs::UAVCommand Step_trajectory_generation(float time_from_start);

        prometheus_msgs::UAVCommand Line_trajectory_generation(float time_from_start);

    private:

        ros::NodeHandle Controller_Test_nh;

        // Circle Parameter
        Eigen::Vector3f circle_center;
        float circle_radius;
        float linear_vel;
        float direction;         //direction = 1 for CCW 逆时针, direction = -1 for CW 顺时针

        // Eight Shape Parameter
        Eigen::Vector3f eight_origin_;
        float radial;
        float eight_omega_;

        // Step
        float step_length;
        float step_interval;
        
};


prometheus_msgs::UAVCommand Controller_Test::Circle_trajectory_generation(float time_from_start)
{
    prometheus_msgs::UAVCommand Circle_trajectory;
    float omega;
    if( circle_radius != 0)
    {
        omega = direction * fabs(linear_vel / circle_radius);
    }else
    {
        omega = 0.0;
    }
    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);

    // cout << GREEN <<  "omega : " << omega  * 180/M_PI <<" [deg/s] " << TAIL <<endl;
    // cout << GREEN <<  "angle : " << angle  * 180/M_PI <<" [deg] " << TAIL <<endl;

    Circle_trajectory.position_ref[0] = circle_radius * cos_angle + circle_center[0];
    Circle_trajectory.position_ref[1] = circle_radius * sin_angle + circle_center[1];
    Circle_trajectory.position_ref[2] = circle_center[2];

    Circle_trajectory.velocity_ref[0] = - circle_radius * omega * sin_angle;
    Circle_trajectory.velocity_ref[1] = circle_radius * omega * cos_angle;
    Circle_trajectory.velocity_ref[2] = 0;

    Circle_trajectory.acceleration_ref[0] = - circle_radius * pow(omega, 2.0) * cos_angle;
    Circle_trajectory.acceleration_ref[1] = - circle_radius * pow(omega, 2.0) * sin_angle;
    Circle_trajectory.acceleration_ref[2] = 0;

    // Circle_trajectory.jerk_ref[0] = circle_radius * pow(omega, 3.0) * sin_angle;
    // Circle_trajectory.jerk_ref[1] = - circle_radius * pow(omega, 3.0) * cos_angle;
    // Circle_trajectory.jerk_ref[2] = 0;

    // Circle_trajectory.snap_ref[0] = circle_radius * pow(omega, 4.0) * cos_angle;
    // Circle_trajectory.snap_ref[1] = circle_radius * pow(omega, 4.0) * sin_angle;
    // Circle_trajectory.snap_ref[2] = 0;

    Circle_trajectory.yaw_ref = 0;
    // Circle_trajectory.yaw_rate_ref = 0;
    // Circle_trajectory.yaw_acceleration_ref = 0;

    return Circle_trajectory;
}

prometheus_msgs::UAVCommand Controller_Test::Line_trajectory_generation(float time_from_start)
{
    prometheus_msgs::UAVCommand Line_trajectory;
    float omega;
    if( circle_radius != 0)
    {
        omega = direction * fabs(linear_vel / circle_radius);
    }else
    {
        omega = 0.0;
    }
    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);

    Line_trajectory.position_ref[0] = 0.0;
    Line_trajectory.position_ref[1] = circle_radius * sin_angle + circle_center[1];
    Line_trajectory.position_ref[2] = circle_center[2];

    Line_trajectory.velocity_ref[0] = 0.0;
    Line_trajectory.velocity_ref[1] = circle_radius * omega * cos_angle;
    Line_trajectory.velocity_ref[2] = 0;

    Line_trajectory.acceleration_ref[0] = 0.0;
    Line_trajectory.acceleration_ref[1] = - circle_radius * pow(omega, 2.0) * sin_angle;
    Line_trajectory.acceleration_ref[2] = 0;

    // Line_trajectory.jerk_ref[0] = circle_radius * pow(omega, 3.0) * sin_angle;
    // Line_trajectory.jerk_ref[1] = - circle_radius * pow(omega, 3.0) * cos_angle;
    // Line_trajectory.jerk_ref[2] = 0;

    // Line_trajectory.snap_ref[0] = circle_radius * pow(omega, 4.0) * cos_angle;
    // Line_trajectory.snap_ref[1] = circle_radius * pow(omega, 4.0) * sin_angle;
    // Line_trajectory.snap_ref[2] = 0;

    Line_trajectory.yaw_ref = 0;
    // Line_trajectory.yaw_rate_ref = 0;
    // Line_trajectory.yaw_acceleration_ref = 0;

    return Line_trajectory;
}


prometheus_msgs::UAVCommand Controller_Test::Eight_trajectory_generation(float time_from_start)
{
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;
    
    float angle = eight_omega_* time_from_start;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);
    
    Eigen::Vector3f eight_radial_ ;
    Eigen::Vector3f eight_axis_ ;
    eight_radial_ << radial, 0.0, 0.0;
    eight_axis_ << 0.0, 0.0, 2.0;

    position = cos_angle * eight_radial_ + sin_angle * cos_angle * eight_axis_.cross(eight_radial_)
                 + (1 - cos_angle) * eight_axis_.dot(eight_radial_) * eight_axis_ + eight_origin_;

    velocity = eight_omega_ * (-sin_angle * eight_radial_ + (pow(cos_angle, 2) - pow(sin_angle, 2)) * eight_axis_.cross(eight_radial_)
                 + (sin_angle) * eight_axis_.dot(eight_radial_) * eight_axis_);

    acceleration << 0.0, 0.0, 0.0;

    prometheus_msgs::UAVCommand Eight_trajectory;

    Eight_trajectory.position_ref[0] = position[0];
    Eight_trajectory.position_ref[1] = position[1];
    Eight_trajectory.position_ref[2] = position[2];

    Eight_trajectory.velocity_ref[0] = velocity[0];
    Eight_trajectory.velocity_ref[1] = velocity[1];
    Eight_trajectory.velocity_ref[2] = velocity[2];

    Eight_trajectory.acceleration_ref[0] = 0;
    Eight_trajectory.acceleration_ref[1] = 0;
    Eight_trajectory.acceleration_ref[2] = 0;

    Eight_trajectory.yaw_ref = 0;

    // to be continued...

    return Eight_trajectory;
}


prometheus_msgs::UAVCommand Controller_Test::Step_trajectory_generation(float time_from_start)
{
    prometheus_msgs::UAVCommand Step_trajectory;

    int i = time_from_start / step_interval;

    if( i%2 == 0)
    {
        Step_trajectory.position_ref[0] = step_length;
    }else 
    {
        Step_trajectory.position_ref[0] = - step_length;
    }

    Step_trajectory.position_ref[1] = 0;
    Step_trajectory.position_ref[2] = 1.0;

    Step_trajectory.velocity_ref[0] = 0;
    Step_trajectory.velocity_ref[1] = 0;
    Step_trajectory.velocity_ref[2] = 0;

    Step_trajectory.acceleration_ref[0] = 0;
    Step_trajectory.acceleration_ref[1] = 0;
    Step_trajectory.acceleration_ref[2] = 0;

    Step_trajectory.yaw_ref = 0;

    return Step_trajectory;
}

// 【打印参数函数】
void Controller_Test::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>Controller_Test Parameter <<<<<<<<<<<<<<<<<<<<<<" << TAIL <<endl;
    cout << GREEN << "Circle Shape  :  " << TAIL <<endl;
    cout << GREEN << "origin        :  " << circle_center[0] <<" [m] "<< circle_center[1] <<" [m] "<< circle_center[2] <<" [m] "<< TAIL <<endl;
    cout << GREEN << "radius        :  " << circle_radius <<" [m] " << endl;
    cout << GREEN << "linear_vel    :  " << linear_vel <<" [m/s] " << endl;
    cout << GREEN << "direction     : "<< direction << endl;

    cout << GREEN << "Eight Shape   :  " << TAIL <<endl;
    cout << GREEN << "origin        :  " << eight_origin_[0] <<" [m] "<< eight_origin_[1] <<" [m] "<< eight_origin_[2] <<" [m] "<< TAIL <<endl;
    cout << GREEN << "radial        :  " << radial  <<" [m] "<< endl;
    cout << GREEN << "angular_vel   :  " << eight_omega_  <<" [rad/s] " << endl;

    cout << GREEN << "Step          :  " << TAIL <<endl;
    cout << GREEN << "step_length   :  " << step_length << TAIL <<endl;
    cout << GREEN << "step_interval :  " << step_interval << " [s] "<< TAIL <<endl;
}


#endif