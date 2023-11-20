#ifndef UGV_ESTIMATOR_H
#define UGV_ESTIMATOR_H

// 头文件
#include <ros/ros.h>
#include <iostream>
#include <bitset>
#include <Eigen/Eigen>
#include <prometheus_msgs/UGVState.h>
#include <prometheus_msgs/UGVCommand.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>




#include "tf2_ros/transform_broadcaster.h"  //发布动态坐标关系
#include "math_utils.h"
#include "printf_utils.h"



// 宏定义
#define TRA_WINDOW 20                     // 发布轨迹长度
#define TIMEOUT_MAX 0.1                     // MOCAP超时阈值
using namespace std;

class UGV_estimator
{
public:
    UGV_estimator(ros::NodeHandle& nh);

    // 变量
    int ugv_id;                                           // 无人车编号
    string ugv_name;                            // 无人车名字(话题前缀)
    bool sim_mode;
    prometheus_msgs::UGVState ugv_state;    // 无人车状态
    prometheus_msgs::UGVCommand ugv_command;
    nav_msgs::Odometry ugv_odom;                // 无人车odom
    std::vector<geometry_msgs::PoseStamped> posehistory_vector_;    // 无人车轨迹容器
    string mesh_resource;
    ros::Time last_mocap_timestamp;                  // mocap时间戳

    std_msgs::UInt8 matlab_ugv_cmd_mode_;
    geometry_msgs::Point matlab_ugv_cmd;
    geometry_msgs::PoseStamped pose_msg;
  

    ros::Timer timer_rviz_pub;
    ros::Timer timer_ugv_state_pub;

    bool mocap_first_time;
    float last_position_x, last_position_y, last_position_z, last_time, now_time, dt;
    // 订阅话题
    ros::Subscriber gazebo_odom_sub;
    ros::Subscriber mocap_pos_sub;
    ros::Subscriber mocap_vel_sub;
    ros::Subscriber battery_sub;
   
    // 发布话题
    ros::Publisher ugv_state_pub;
    ros::Publisher ugv_odom_pub;
    ros::Publisher ugv_mesh_pub;
    ros::Publisher trajectory_pub;
    ros::Publisher mocap_matlab_pub;
    
    enum MatlabUGVState
    {
        HOLD = 0,
        Direct_Control_BODY = 1,
        Direct_Control_ENU = 2,
        Point_Control = 3,
        Path_Control = 4,
        Test = 5
    };


private:
    void mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void battery_cb(const std_msgs::Float32::ConstPtr &msg);
    void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg);
    float get_time_in_sec(const ros::Time& begin_time);
    void timercb_ugv_state(const ros::TimerEvent &e);
    void timercb_rviz(const ros::TimerEvent &e);
    void matlab_ugv_cmd_mode_cb(const std_msgs::UInt8::ConstPtr &msg);
    void matlab_ugv_cmd_cb(const geometry_msgs::Point::ConstPtr &msg);
};



#endif
