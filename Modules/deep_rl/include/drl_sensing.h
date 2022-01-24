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
#include <prometheus_drl/agent_state.h>

#include "occupy_map_2d.h"
#include "printf_utils.h"

#include "angles/angles.h"
#include <tf2/utils.h> // getYaw
#include<math.h>

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
    int swarm_num;
    // 智能体前缀  
    string agent_prefix;   
    // 智能体名字                             
    string agent_name;    
    // 是否仿真模式
    bool sim_mode;    
    // 智能体编号                         
    int agent_id;                                     
    // 智能体高度
    double agent_height;
    float block_size;
    int matrix_size;
    float inv_block_size;
    int block_num;

    prometheus_drl::agent_state agent_state;
    Eigen::MatrixXd agent_goal;
    
    // 【订阅】
    ros::Subscriber odom_sub;
    ros::Subscriber scan_sub;
    
    ros::Subscriber nei_odom_sub[21];
    // 其他智能体位置
    nav_msgs::Odometry nei_odom[21];
    Eigen::Vector3d odom_nei[21];
    bool get_nei_odom[21];

    // 【发布】
    ros::Publisher agent_state_pub;
    // 【定时器】
    ros::Timer pub_agent_state_timer;
    ros::Timer update_other_agent_pos_timer;
    // 占据图类
    Occupy_map::Ptr Occupy_map_ptr;

    // 智能体里程计信息
    nav_msgs::Odometry agent_odom;
    bool odom_ready;
    bool sensor_ready;
    
    void pub_agent_state_cb(const ros::TimerEvent& e);
    void update_other_agent_pos_cb(const ros::TimerEvent& e);
    void odom_cb(const nav_msgs::OdometryConstPtr &msg);
    void scan_cb(const sensor_msgs::LaserScanConstPtr &msg);
    void nei_odom_cb(const nav_msgs::Odometry::ConstPtr& odom, int id);

};

}

#endif
