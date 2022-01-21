#ifndef LOCAL_PLANNING_H
#define LOCAL_PLANNING_H

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

#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/Message.h"
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/ControlCommand.h"

#include "apf.h"
#include "vfh.h"
#include "tools.h"
#include "message_utils.h"

using namespace std;
#define NODE_NAME "Local_Planner [main]"

#define MIN_DIS 0.1

namespace Local_Planning
{

extern ros::Publisher message_pub;

class Local_Planner
{

private:

    ros::NodeHandle local_planner_nh;

    // 参数
    int algorithm_mode;
    int lidar_model;
    bool is_2D;
    double max_planning_vel;
    double fly_height_2D;
    double safe_distance;
    bool sim_mode;
    bool map_groundtruth;

    // 订阅无人机状态、目标点、传感器数据（生成地图）
    ros::Subscriber goal_sub;
    ros::Subscriber drone_state_sub;

    ros::Subscriber local_point_clound_sub;
    ros::Subscriber swith_sub;

    // 发布控制指令
    ros::Publisher command_pub,rviz_vel_pub;
    ros::Timer mainloop_timer,control_timer;

    // 局部避障算法 算子
    local_planning_alg::Ptr local_alg_ptr;

    prometheus_msgs::DroneState _DroneState;
    nav_msgs::Odometry Drone_odom;
    prometheus_msgs::ControlCommand Command_Now;  

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

    // 打印的提示消息
    string message;

    // 五种状态机
    enum EXEC_STATE
    {
        WAIT_GOAL,
        PLANNING,
        TRACKING,
        LANDING,
    };
    EXEC_STATE exec_state;

    sensor_msgs::PointCloud2ConstPtr  local_map_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
    pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;

    void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg);
    void drone_state_cb(const prometheus_msgs::DroneStateConstPtr &msg);
    void localcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg);
    void mainloop_cb(const ros::TimerEvent& e);
    void control_cb(const ros::TimerEvent& e);

public:

    Local_Planner(void):
        local_planner_nh("~") {}~Local_Planner(){}

    double obs_distance;
    double att_distance;

    Eigen::Matrix<double, 3, 1> total_force;

    void init(ros::NodeHandle& nh);

};



}
#endif 
