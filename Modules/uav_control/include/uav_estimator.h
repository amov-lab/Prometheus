#ifndef UAV_ESTIMATOR_H
#define UAV_ESTIMATOR_H

// 头文件
#include <ros/ros.h>
#include <iostream>
#include <bitset>
#include <Eigen/Eigen>
#include <mavros/frame_tf.h>
#include <GeographicLib/Geocentric.hpp>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/TextInfo.h>
#include <prometheus_msgs/OffsetPose.h>
#include <prometheus_msgs/GPSData.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/BatteryState.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>

#include "tf2_ros/transform_broadcaster.h"  //发布动态坐标关系
#include "math_utils.h"
#include "printf_utils.h"


using namespace std;

// 宏定义
#define TRA_WINDOW 40                // 发布轨迹长度
#define MOCAP_TIMEOUT 0.1                   
#define GAZEBO_TIMEOUT 0.1                    
#define T265_TIMEOUT 0.1                     

class UAV_estimator
{
    public:
        UAV_estimator(ros::NodeHandle& nh);

        // 订阅话题
        ros::Subscriber px4_state_sub;
        ros::Subscriber px4_battery_sub;
        ros::Subscriber px4_position_sub;
        ros::Subscriber px4_velocity_sub;
        ros::Subscriber px4_attitude_sub;
        ros::Subscriber mocap_sub;
        ros::Subscriber gazebo_sub;
        ros::Subscriber fake_odom_sub;
        ros::Subscriber px4_global_position_sub;
        ros::Subscriber gps_status_sub;
        ros::Subscriber set_local_pose_offset_sub;
        // 发布话题
        ros::Publisher uav_state_pub;
        ros::Publisher px4_vision_pose_pub;
        ros::Publisher uav_odom_pub;
        ros::Publisher uav_trajectory_pub;
        ros::Publisher uav_mesh_pub;
        ros::Publisher gps_status_pub;
        ros::Publisher ground_station_info_pub;
        ros::Publisher local_pose_offset_pub;
        // 定时器
        ros::Timer timer_px4_vision_pub;
        ros::Timer timer_uav_state_pub;
        ros::Timer timer_rviz_pub;

        prometheus_msgs::UAVState uav_state;    // 无人机状态
        nav_msgs::Odometry uav_odom;                 // 无人机里程计
        std::vector<geometry_msgs::PoseStamped> pos_vector;    // 无人机轨迹容器,用于rviz显示
        geometry_msgs::PoseStamped vision_pose;        // vision_pose for px4
        prometheus_msgs::TextInfo text_info;
        prometheus_msgs::OffsetPose offset_pose;

        geometry_msgs::PoseStamped mocap_pose;         // mocap pose
        geometry_msgs::PoseStamped gazebo_pose;        // gazebo pose
        geometry_msgs::PoseStamped t265_pose;          // gazebo pose
        int odom_state,last_odom_state{9};
        
        ros::Time get_mocap_stamp{0};
        ros::Time get_gazebo_stamp{0};
        ros::Time get_t265_stamp{0};

        // 基本变量
        int uav_id;                   // 无人机编号
        string uav_name;                // 无人机名字
        string node_name;
        int location_source;    
        float maximum_safe_vel_xy;
        float maximum_safe_vel_z;
        float maximum_vel_error_for_vision;
        int last_gps_status;
        bool fake_odom;                 // Gazebo是否使用fake_odom
        bool uav_state_update{false};
        bool vision_pose_error{false};

        void printf_uav_state();

    private:

        void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void fake_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void t265_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void px4_state_cb(const mavros_msgs::State::ConstPtr &msg);
        void px4_battery_cb(const sensor_msgs::BatteryState::ConstPtr &msg);
        void px4_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void px4_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void px4_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void px4_att_cb(const sensor_msgs::Imu::ConstPtr& msg);
        void gps_status_cb(const mavros_msgs::GPSRAW::ConstPtr& msg);
        void set_local_pose_offset_cb(const prometheus_msgs::GPSData::ConstPtr& msg);

        void timercb_pub_vision_pose(const ros::TimerEvent &e);
        void timercb_pub_uav_state(const ros::TimerEvent &e);
        void timercb_rviz(const ros::TimerEvent &e);
        void check_uav_state();
        int check_uav_odom();
        
        void printf_gps_status();
        void printf_param();
};


#endif