#ifndef FAKE_UGV_H
#define FAKE_UGV_H

#include <ros/ros.h>

#include <prometheus_msgs/UgvState.h>
#include <prometheus_msgs/UgvCommand.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelState.h>

#include "math_utils.h"
#include "printf_utils.h"

using namespace std;

class Fake_UGV
{
    public:
        Fake_UGV(){};

        void init(ros::NodeHandle& nh, int id, Eigen::Vector3d init_pos, double init_yaw);
        Eigen::Vector3d get_ugv_pos();
        gazebo_msgs::ModelState get_model_state();

    private:
        // 订阅
        ros::Subscriber vel_cmd_sub;
        ros::Publisher fake_odom_pub;
        ros::Timer fake_odom_timer;
        ros::Timer fake_odom_pub_timer;
        ros::Timer debug_timer;

        int agent_id;                                    
        string model_name;                             
        string node_name;
        int car_model;
        int fake_mode;
        bool get_vel_cmd;
        bool ugv_state_update;
        double delta_time;

        nav_msgs::Odometry fake_odom;
        gazebo_msgs::ModelState gazebo_model_state;

        // 控制信号 - 来自controller
        Eigen::Vector3d vel_sp_body;
        Eigen::Vector3d vel_sp_enu;
        double yaw_rate_sp;

        enum CAR_MODEL
        {
            POINT,          
            DIFFERENTIAL_WHELL
        };

        struct UGV_state
        {
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            Eigen::Vector3d euler;
            geometry_msgs::Quaternion quat;
            Eigen::Quaterniond quat2; 
        } ugv_state;

        struct Control_Param
        {
            float k_pos;
            float k_vel;
        } control_param;

        // 回调函数
        void vel_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg);
        void fake_odom_process(const ros::TimerEvent& e);
        void fake_odom_pub_cb(const ros::TimerEvent& e);
        void debug_cb(const ros::TimerEvent& e);
};

#endif