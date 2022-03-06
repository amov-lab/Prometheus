#ifndef GLOBAL_PLANNER
#define GLOBAL_PLANNER

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>

#include "A_star.h"
#include "occupy_map.h"
#include "printf_utils.h"
#include "global_planner_utils.h"
using namespace std;

namespace GlobalPlannerNS
{
    class GlobalPlanner
    {
    public:
        GlobalPlanner(ros::NodeHandle &nh);
        ros::NodeHandle global_planner_nh;

    private:
        // 参数
        int uav_id;
        double fly_height;
        double safe_distance;
        double time_per_path;
        int map_input_source;
        double replan_time;
        bool consider_neighbour;
        bool sim_mode;

        // 订阅无人机状态、目标点、传感器数据（生成地图）
        ros::Subscriber goal_sub;
        ros::Subscriber uav_state_sub;
        // 支持2维激光雷达、3维激光雷达、D435i等实体传感器
        // 支持直接输入全局已知点云
        ros::Subscriber Gpointcloud_sub;
        ros::Subscriber Lpointcloud_sub;
        ros::Subscriber laserscan_sub;

        // 发布控制指令
        ros::Publisher uav_cmd_pub;
        ros::Publisher path_cmd_pub;
        ros::Timer mainloop_timer;
        ros::Timer track_path_timer;
        ros::Timer safety_timer;
        // A星规划器
        Astar::Ptr Astar_ptr;

        prometheus_msgs::UAVState uav_state;      // 无人机状态
        nav_msgs::Odometry uav_odom;
        Eigen::Vector3d uav_pos;  // 无人机位置
        Eigen::Vector3d uav_vel;  // 无人机速度
        Eigen::Quaterniond uav_quat; // 无人机四元数
        double uav_yaw;

        prometheus_msgs::UAVCommand uav_command; 

        nav_msgs::Path path_cmd;
        double distance_walked;
        

        double distance_to_goal;

        // 规划器状态
        bool odom_ready;
        bool drone_ready;
        bool sensor_ready;
        bool goal_ready;
        bool is_safety;
        bool is_new_path;
        bool path_ok;
        int start_point_index;
        int Num_total_wp;
        int cur_id;

        // 规划初始状态及终端状态
        Eigen::Vector3d start_pos, start_vel, start_acc, goal_pos, goal_vel;

        float desired_yaw;

        ros::Time tra_start_time;
        float tra_running_time;

        // 五种状态机
        enum EXEC_STATE
        {
            WAIT_GOAL,
            PLANNING,
            TRACKING,
            LANDING,
        };
        EXEC_STATE exec_state;

        // 回调函数
        void goal_cb(const geometry_msgs::PoseStampedConstPtr &msg);
        void uav_state_cb(const prometheus_msgs::UAVStateConstPtr &msg);
        void Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
        void Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
        void laser_cb(const sensor_msgs::LaserScanConstPtr &msg);

        void safety_cb(const ros::TimerEvent &e);
        void mainloop_cb(const ros::TimerEvent &e);
        void track_path_cb(const ros::TimerEvent &e);

        // 【获取当前时间函数】 单位：秒
        float get_time_in_sec(const ros::Time &begin_time);

        int get_start_point_id(void);


    };

} // namespace GlobalPlannerNS

#endif