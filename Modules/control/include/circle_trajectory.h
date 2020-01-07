/***************************************************************************************************************************
* circle_trajectory.h
*
* Author: Qyp
*
* Update Time: 2019.6.28
*
* Introduction:  Circle trajectory generation code
*         1. Generating the circle trajectory
*         2. Parameter: center, radius, linear_vel, time_total, direction
*         3. Input: time_from_start
*         4. Output: position, velocity, acceleration, jerk, snap
***************************************************************************************************************************/
#ifndef CIRCLE_TRAJECTORY_H
#define CIRCLE_TRAJECTORY_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include <prometheus_msgs/PositionReference.h>
#include <command_to_mavros.h>

using namespace std;

class Circle_Trajectory
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        Circle_Trajectory(void):
            Circle_Trajectory_nh("~")
        {
            Circle_Trajectory_nh.param<float>("Circle_Trajectory/Center_x", center[0], 0.0);
            Circle_Trajectory_nh.param<float>("Circle_Trajectory/Center_y", center[1], 0.0);
            Circle_Trajectory_nh.param<float>("Circle_Trajectory/Center_z", center[2], 1.0);
            Circle_Trajectory_nh.param<float>("Circle_Trajectory/radius", radius, 1.0);
            Circle_Trajectory_nh.param<float>("Circle_Trajectory/linear_vel", linear_vel, 0.5);
            Circle_Trajectory_nh.param<float>("Circle_Trajectory/time_total", time_total, 10.0);
            Circle_Trajectory_nh.param<float>("Circle_Trajectory/direction", direction, 1.0);
        }
        
        // Parameter
        Eigen::Vector3f center;
        float radius;
        float linear_vel;
        float time_total;
        float direction;         //direction = 1 for CCW 逆时针, direction = -1 for CW 顺时针

        //Printf the Circle_Trajectory parameter
        void printf_param();

        //Printf the Circle_Trajectory result
        void printf_result(prometheus_msgs::PositionReference& Circle_trajectory);

        //Circle_Trajectory Calculation [Input: time_from_start; Output: Circle_trajectory;]
        prometheus_msgs::PositionReference Circle_trajectory_generation(float time_from_start);

    private:

        ros::NodeHandle Circle_Trajectory_nh;
};


prometheus_msgs::PositionReference Circle_Trajectory::Circle_trajectory_generation(float time_from_start)
{
    prometheus_msgs::PositionReference Circle_trajectory;
    float omega;
    if( radius != 0)
    {
        omega = direction * fabs(linear_vel / radius);
    }else
    {
        omega = 0.0;
    }
    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);

    // cout << "omega : " << omega  * 180/M_PI <<" [deg/s] " <<endl;
    // cout << "angle : " << angle  * 180/M_PI <<" [deg] " <<endl;

    Circle_trajectory.header.stamp = ros::Time::now();

    Circle_trajectory.time_from_start = time_from_start;

    Circle_trajectory.Move_mode  = prometheus_msgs::PositionReference::XYZ_POS;

    Circle_trajectory.position_ref[0] = radius * cos_angle + center[0];
    Circle_trajectory.position_ref[1] = radius * sin_angle + center[1];
    Circle_trajectory.position_ref[2] = center[2];

    Circle_trajectory.velocity_ref[0] = - radius * omega * sin_angle;
    Circle_trajectory.velocity_ref[1] = radius * omega * cos_angle;
    Circle_trajectory.velocity_ref[2] = 0;

    Circle_trajectory.acceleration_ref[0] = - radius * pow(omega, 2.0) * cos_angle;
    Circle_trajectory.acceleration_ref[1] = - radius * pow(omega, 2.0) * sin_angle;
    Circle_trajectory.acceleration_ref[2] = 0;

    // Circle_trajectory.jerk_ref[0] = radius * pow(omega, 3.0) * sin_angle;
    // Circle_trajectory.jerk_ref[1] = - radius * pow(omega, 3.0) * cos_angle;
    // Circle_trajectory.jerk_ref[2] = 0;

    // Circle_trajectory.snap_ref[0] = radius * pow(omega, 4.0) * cos_angle;
    // Circle_trajectory.snap_ref[1] = radius * pow(omega, 4.0) * sin_angle;
    // Circle_trajectory.snap_ref[2] = 0;

    Circle_trajectory.yaw_ref = 0;
    // Circle_trajectory.yaw_rate_ref = 0;
    // Circle_trajectory.yaw_acceleration_ref = 0;

    return Circle_trajectory;
}


void Circle_Trajectory::printf_result(prometheus_msgs::PositionReference& Circle_trajectory)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>> Circle_Trajectory <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);

    cout << "time_from_start : " << Circle_trajectory.time_from_start<< " [s] " <<endl;

    cout << "position  [X Y Z] : " << Circle_trajectory.position_ref[0] << " [m] "<< Circle_trajectory.position_ref[1]<<" [m] "<<Circle_trajectory.position_ref[2]<<" [m] "<<endl;

    cout << "velocity [X Y Z] : " << Circle_trajectory.velocity_ref[0] << " [m/s] "<< Circle_trajectory.velocity_ref[1]<<" [m/s] "<<Circle_trajectory.velocity_ref[2]<<" [m/s] "<<endl;

    cout << "acceleration [X Y Z] : " << Circle_trajectory.acceleration_ref[0] << " [m/s^2] "<< Circle_trajectory.acceleration_ref[1]<<" [m/s^2] "<< Circle_trajectory.acceleration_ref[2]<<" [m/s^2] "<<endl;

    // cout << "jerk [X Y Z] : " << Circle_trajectory.jerk_ref[0] << " [m/s^3] "<< Circle_trajectory.jerk_ref[1]<<" [m/s^3] "<<Circle_trajectory.jerk_ref[2]<<" [m/s^3] "<<endl;

    // cout << "snap [X Y Z] : " << Circle_trajectory.snap_ref[0] << " [m/s^4] "<< Circle_trajectory.snap_ref[1]<<" [m/s^4] "<<Circle_trajectory.snap_ref[2]<<" [m/s^4] "<<endl;

}

// 【打印参数函数】
void Circle_Trajectory::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Circle_Trajectory Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"center :  "<< center <<endl;
    cout <<"radius :  "<< radius << endl;
    cout <<"linear_vel : "<< linear_vel << endl;
    cout <<"time_total : "<< time_total << endl;
    cout <<"direction : "<< direction << endl;
    //direction = 1 for CCW 逆时针, direction = -1 for CW 顺时针
}



#endif
