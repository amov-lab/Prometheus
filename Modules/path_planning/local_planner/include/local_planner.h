#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>

#include "apf.h"
#include "vfh.h"
#include "local_planner_utils.h"

using namespace std;

namespace LocalPlannerNS
{
    class LocalPlanner
    {
    public:
        LocalPlanner(ros::NodeHandle &nh);
        ros::NodeHandle local_planner_nh;

    private:
        // 参数
        int uav_id;
        int algorithm_mode;
        int lidar_model;
        double max_planning_vel;
        double fly_height;
        double safe_distance;
        bool sim_mode;
        bool map_groundtruth;

        // 订阅无人机状态、目标点、传感器数据（生成地图）
        ros::Subscriber goal_sub;
        ros::Subscriber uav_state_sub;

        ros::Subscriber local_point_cloud_sub;

        // 发布控制指令
        ros::Publisher uav_cmd_pub;
        ros::Publisher rviz_vel_pub;
        ros::Timer mainloop_timer;
        ros::Timer control_timer;

        // 局部避障算法 算子
        local_planner_alg::Ptr local_alg_ptr;

        prometheus_msgs::UAVState uav_state;      // 无人机状态
        nav_msgs::Odometry uav_odom;
        Eigen::Vector3d uav_pos;  // 无人机位置
        Eigen::Vector3d uav_vel;  // 无人机速度
        Eigen::Quaterniond uav_quat; // 无人机四元数
        double uav_yaw;

        prometheus_msgs::UAVCommand uav_command; 

        double distance_to_goal;

        // 规划器状态
        bool odom_ready;
        bool drone_ready;
        bool sensor_ready;
        bool goal_ready;
        bool is_safety;
        bool path_ok;

        // 规划初始状态及终端状态
        Eigen::Vector3d start_pos, start_vel, start_acc, goal_pos, goal_vel;

        int planner_state;
        Eigen::Vector3d desired_vel;
        float desired_yaw;

        geometry_msgs::Point vel_rviz;

        // 五种状态机
        enum EXEC_STATE
        {
            WAIT_GOAL,
            PLANNING,
            TRACKING,
            LANDING,
        };
        EXEC_STATE exec_state;

        sensor_msgs::PointCloud2ConstPtr local_map_ptr_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
        pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;

        void goal_cb(const geometry_msgs::PoseStampedConstPtr &msg);
        void uav_state_cb(const prometheus_msgs::UAVStateConstPtr &msg);
        void pcl_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
        void laserscan_cb(const sensor_msgs::LaserScanConstPtr &msg);
        void mainloop_cb(const ros::TimerEvent &e);
        void control_cb(const ros::TimerEvent &e);
    };

} // namespace LocalPlannerNS
#endif
