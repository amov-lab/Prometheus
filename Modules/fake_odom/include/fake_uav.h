#ifndef FAKE_UAV_H
#define FAKE_UAV_H

#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/ModelState.h>
#include "quadrotor_msgs/PositionCommand.h"

#include "uav_utils/geometry_utils.h"
#include "math_utils.h"
#include "printf_utils.h"
#include <Quadrotor_dynamics.h>


using namespace std;

class Fake_UAV
{
    public:
        
        Fake_UAV(){};
        void init(ros::NodeHandle& nh, int id, Eigen::Vector3d init_pos, double init_yaw);
        Eigen::Vector3d get_uav_pos();
        gazebo_msgs::ModelState get_model_state();

    private:
        // 订阅
        ros::Subscriber pos_cmd_sub;
        ros::Subscriber att_cmd_sub;
        ros::Subscriber ego_cmd_sub;
        ros::Publisher fake_odom_pub;
        ros::Timer fake_odom_timer;
        ros::Timer fake_odom_pub_timer;
        ros::Timer debug_timer;

        int agent_id;                                     // 无人机编号
        string model_name;                             
        string node_name;
        int fake_mode;
        bool get_pos_cmd;
        bool get_att_cmd;
        bool get_ego_cmd;
        bool uav_state_update;
        double delta_time;            // 妙

        // QuadrotorSimulator::Quadrotor quad;
        nav_msgs::Odometry fake_odom;
        gazebo_msgs::ModelState gazebo_model_state;

        // 输入来源优先级：ego轨迹追踪 > 姿态控制（暂时支持较差）> 位置控制（含位置速度复合控制）
        enum FAKE_MODE
        {
            POS_CONTROL_MODE,          
            ATT_CONTROL_MODE,
            RPM_CONTROL_MODE
        };

        struct Disturbance
        {
            Eigen::Vector3d f;
            Eigen::Vector3d m;
        } disturbance;

        struct POS_Command
        {
            Eigen::Vector3d pos_sp;
            Eigen::Vector3d vel_sp;
            Eigen::Vector3d acc_sp;
            double yaw_sp;
        } pos_cmd;

        struct ATT_Command
        {
            Eigen::Vector3d euler_sp;
            double throttle_sp;
        } att_cmd;

        struct UAV_state
        {
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            Eigen::Vector3d acc;
            Eigen::Vector3d euler;
            geometry_msgs::Quaternion quat;
            Eigen::Quaterniond quat2; 
            Eigen::Matrix3d Rb;
            Eigen::Vector3d thrust_enu;
        } uav_state;

        struct Control_Output
        {
            Eigen::Vector3d u_cmd;      // 可以理解为期望加速度
            Eigen::Vector3d thrust_body;
            Eigen::Vector3d thrust_enu;
            Eigen::Vector3d euler;
            double yaw;
            double rpm[4];
        } control;

        struct Control_Param
        {
            float quad_mass;
            float gravity;
            float k_pos;
            float k_vel;
            float tilt_angle_max;
            float hover_per;
        } control_param;

        // 回调函数
        void pos_cmd_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
        void att_cmd_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
        void ego_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
        void fake_odom_process(const ros::TimerEvent& e);
        void fake_odom_pub_cb(const ros::TimerEvent& e);
        void debug_cb(const ros::TimerEvent& e);

        // 功能函数
        void fake_pos_control();
        void publish_fake_odom();
        void attitude_control();
        void mixer();
};

#endif