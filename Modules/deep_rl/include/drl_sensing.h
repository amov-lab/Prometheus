#ifndef DRL_SENSING_H
#define DRL_SENSING_H

#include <ros/ros.h>
#include <boost/format.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <prometheus_drl/ugv_state.h>

#include "occupy_map_2d.h"
#include "printf_utils.h"

#include "angles/angles.h"
#include <tf2/utils.h> // getYaw

using namespace std;

namespace drl_ns
{

class drl_sensing
{
public:
    drl_sensing(){};
    void init(ros::NodeHandle& nh, int id, Eigen::MatrixXd goal);
    void reset(Eigen::MatrixXd goal);
    void printf_cb();

private:
    // 集群数量
    int swarm_num_ugv;     
    // 无人车名字                             
    string ugv_name;    
    // 是否仿真模式
    bool sim_mode;    
    // 无人车编号                         
    int ugv_id;                                     
    // 无人车高度
    double ugv_height;
    float block_size;
    int matrix_size;
    float inv_block_size;
    int block_num;

    prometheus_drl::ugv_state ugv_state;
    Eigen::MatrixXd ugv_goal;
    
    // 【订阅】
    ros::Subscriber odom_sub;
    ros::Subscriber scan_sub;
    
    ros::Subscriber nei_odom_sub[21];
    // 其他无人车位置
    nav_msgs::Odometry nei_odom[21];
    Eigen::Vector3d odom_nei[21];
    bool get_nei_odom[21];

    // 【发布】
    ros::Publisher ugv_state_pub;
    // 【定时器】
    ros::Timer pub_ugv_state_timer;
    ros::Timer update_other_ugv_pos_timer;
    // 占据图类
    Occupy_map::Ptr Occupy_map_ptr;

    // 无人车里程计信息
    nav_msgs::Odometry ugv_odom;
    bool odom_ready;
    bool sensor_ready;
    
    void pub_ugv_state_cb(const ros::TimerEvent& e);
    void update_other_ugv_pos_cb(const ros::TimerEvent& e);
    void odom_cb(const nav_msgs::OdometryConstPtr &msg);
    void scan_cb(const sensor_msgs::LaserScanConstPtr &msg);
    void nei_odom_cb(const nav_msgs::Odometry::ConstPtr& odom, int id);

};

}

#endif
