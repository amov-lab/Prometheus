#ifndef DRL_ACTUATOR_H
#define DRL_ACTUATOR_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <prometheus_drl/ugv_move_cmd.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2_ros/transform_broadcaster.h"  //发布动态坐标关系
#include <visualization_msgs/Marker.h>

#include "printf_utils.h"

using namespace std;

namespace drl_ns
{

class drl_actuator
{
    public:
        drl_actuator(){};
        void init(ros::NodeHandle& nh, int id, Eigen::Vector3d init_pos, double init_yaw);
        void reset(Eigen::Vector3d _init_pos, double _init_yaw);
        void printf_cb();

        Eigen::Vector3d get_ugv_pos();
        gazebo_msgs::ModelState get_model_state();

    private:
        // 订阅
        ros::Subscriber move_cmd_sub;
        ros::Publisher fake_odom_pub;
        ros::Publisher ugv_mesh_pub;
        ros::Timer fake_odom_pub_timer;
        ros::Timer debug_timer;

        int agent_id;                                    
        string model_name;                             
        string node_name;
        int fake_mode;
        bool get_move_cmd;
        bool ugv_state_update;
        int cmd_id;
        double block_size;

        nav_msgs::Odometry fake_odom;
        gazebo_msgs::ModelState gazebo_model_state;
        prometheus_drl::ugv_move_cmd action;
        Eigen::Vector3d init_pos;

        struct UGV_state
        {
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            Eigen::Vector3d euler;
            geometry_msgs::Quaternion quat;
            Eigen::Quaterniond quat2; 
        } ugv_state;

        // 回调函数
        void move_cmd_cb(const prometheus_drl::ugv_move_cmd::ConstPtr& msg);
        void fake_odom_pub_cb(const ros::TimerEvent& e);
        Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy);

};
}
#endif