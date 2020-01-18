/***************************************************************************************************************************
* controller_test.h
*
* Author: Qyp
*
* Update Time: 2020.1.10
*
***************************************************************************************************************************/
#ifndef CONTROLLER_TEST_H
#define CONTROLLER_TEST_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include <prometheus_control_utils.h>
#include <prometheus_msgs/PositionReference.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


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
            Controller_Test_nh.param<float>("Controller_Test/Circle/circle_radius", circle_radius, 1.0);
            Controller_Test_nh.param<float>("Controller_Test/Circle/linear_vel", linear_vel, 0.5);
            Controller_Test_nh.param<float>("Controller_Test/Circle/direction", direction, 1.0);

            Controller_Test_nh.param<float>("Controller_Test/Step/step_length", step_length, 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Step/step_interval", step_interval, 0.0);

            Controller_Test_nh.param<float>("Quad/mass", Quad_MASS, 1.0);
            Controller_Test_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);

            ref_trajectory_pub = Controller_Test_nh.advertise<nav_msgs::Path>("/prometheus/reference_trajectory", 10);
            ref_pose_pub = Controller_Test_nh.advertise<geometry_msgs::PoseStamped>("/prometheus/reference_pose", 10);
            
            reference_trajectory.header.stamp = ros::Time::now();
            reference_trajectory.header.frame_id = "map";
        }
        
        float Quad_MASS;
        float tilt_max;

        // Circle Parameter
        Eigen::Vector3f circle_center;
        float circle_radius;
        float linear_vel;
        float direction;         //direction = 1 for CCW 逆时针, direction = -1 for CW 顺时针

        // Step
        float step_length;
        float step_interval;

        //Printf the Controller_Test parameter
        void printf_param();

        //Printf the Controller_Test result
        void printf_result(prometheus_msgs::PositionReference& Circle_trajectory);

        //Controller_Test Calculation [Input: time_from_start; Output: Circle_trajectory;]
        prometheus_msgs::PositionReference Circle_trajectory_generation(float time_from_start);

        prometheus_msgs::PositionReference Eight_trajectory_generation(float time_from_start);

        prometheus_msgs::PositionReference Step_trajectory_generation(float time_from_start);

    private:

        ros::NodeHandle Controller_Test_nh;

        ros::Publisher ref_pose_pub;
        ros::Publisher ref_trajectory_pub;
        
        geometry_msgs::PoseStamped reference_pose;
        nav_msgs::Path reference_trajectory;
};


prometheus_msgs::PositionReference Controller_Test::Circle_trajectory_generation(float time_from_start)
{
    prometheus_msgs::PositionReference Circle_trajectory;
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

    // cout << "omega : " << omega  * 180/M_PI <<" [deg/s] " <<endl;
    // cout << "angle : " << angle  * 180/M_PI <<" [deg] " <<endl;

    Circle_trajectory.header.stamp = ros::Time::now();

    Circle_trajectory.time_from_start = time_from_start;

    Circle_trajectory.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;

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

    reference_pose.header.stamp = ros::Time::now();
    reference_pose.header.frame_id = "map";

    reference_pose.pose.position.x = Circle_trajectory.position_ref[0];
    reference_pose.pose.position.y = Circle_trajectory.position_ref[1];
    reference_pose.pose.position.z = Circle_trajectory.position_ref[2];

    // 待补充：增加期望加速度转期望角度
    Eigen::Vector3d accel_sp;
    Eigen::Vector3d thrust_sp;
    Eigen::Vector3d throttle_sp;
    accel_sp[0] = Circle_trajectory.acceleration_ref[0];
    accel_sp[1] = Circle_trajectory.acceleration_ref[1];
    accel_sp[2] = Circle_trajectory.acceleration_ref[2];

    thrust_sp =  prometheus_control_utils::accelToThrust(accel_sp, Quad_MASS, tilt_max);
    throttle_sp = prometheus_control_utils::thrustToThrottle(thrust_sp);
    prometheus_msgs::AttitudeReference _AttitudeReference;           //位置控制器输出，即姿态环参考量
    _AttitudeReference = prometheus_control_utils::ThrottleToAttitude(throttle_sp, Circle_trajectory.yaw_ref);

    reference_pose.pose.orientation = _AttitudeReference.desired_att_q;
    
    reference_trajectory.poses.push_back(reference_pose);

    ref_pose_pub.publish(reference_pose);
    ref_trajectory_pub.publish(reference_trajectory);

    return Circle_trajectory;
}

prometheus_msgs::PositionReference Controller_Test::Eight_trajectory_generation(float time_from_start)
{
    prometheus_msgs::PositionReference Eight_trajectory;

    Eight_trajectory.header.stamp = ros::Time::now();

    Eight_trajectory.time_from_start = time_from_start;

    Eight_trajectory.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    double traj_omega_ = 0.4;
    double angle = traj_omega_* time_from_start;
    const double cos_angle = cos(angle);
    const double sin_angle = sin(angle);
    Eigen::Vector3d traj_origin_ ;
    Eigen::Vector3d traj_radial_ ;
    Eigen::Vector3d traj_axis_ ;
    traj_origin_ << 0.0, 0.0, 2.0;
    traj_radial_ << 2.0, 0.0, 0.0;
    traj_axis_ << 0.0, 0.0, 2.0;

    position = cos_angle * traj_radial_ + sin_angle * cos_angle * traj_axis_.cross(traj_radial_)
                 + (1 - cos_angle) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;

    velocity = traj_omega_ * (-sin_angle * traj_radial_ + (pow(cos_angle, 2) - pow(sin_angle, 2)) * traj_axis_.cross(traj_radial_)
                 + (sin_angle) * traj_axis_.dot(traj_radial_) * traj_axis_);

    acceleration << 0.0, 0.0, 0.0;

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


prometheus_msgs::PositionReference Controller_Test::Step_trajectory_generation(float time_from_start)
{
    prometheus_msgs::PositionReference Step_trajectory;

    Step_trajectory.header.stamp = ros::Time::now();

    Step_trajectory.time_from_start = time_from_start;

    Step_trajectory.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;

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

void Controller_Test::printf_result(prometheus_msgs::PositionReference& Circle_trajectory)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>> Controller_Test <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

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
void Controller_Test::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Controller_Test Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"circle_center :  "<< circle_center <<endl;
    cout <<"circle_radius :  "<< circle_radius << endl;
    cout <<"linear_vel : "<< linear_vel << endl;
    cout <<"direction : "<< direction << endl;
    //direction = 1 for CCW 逆时针, direction = -1 for CW 顺时针
}



#endif
