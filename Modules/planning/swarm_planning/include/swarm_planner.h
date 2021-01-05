#ifndef SWARM_PLANNER
#define SWARM_PLANNER

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <prometheus_msgs/SwarmCommand.h>
#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/Message.h"
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/ControlCommand.h"

#include "A_star.h"
#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"

using namespace std;

#define NODE_NAME "Swarm_Planner [main]"

#define MIN_DIS 0.1

namespace Swarm_Planning
{

extern ros::Publisher message_pub;

class Swarm_Planner
{
private:

    ros::NodeHandle swarm_planner_nh;

    // 参数

    int swarm_num;
    string uav_name;
    int uav_id,neighbour_id1,neighbour_id2;
    string neighbour_name1,neighbour_name2;
    bool is_2D;
    double fly_height_2D;
    double safe_distance;
    double time_per_path;
    int map_input;
    double replan_time;
    bool consider_neighbour;
    bool sim_mode;
    bool map_groundtruth;
    double min_dis;

    // 本机位置
    // 邻机位置
    // 根据不同的输入（激光雷达输入、相机输入等）生成occupymap
    // 调用路径规划算法 生成路径
    // 调用轨迹优化算法 规划轨迹

    // 订阅无人机状态、目标点、传感器数据（生成地图）
    ros::Subscriber goal_sub;
    ros::Subscriber drone_state_sub;

    ros::Subscriber  nei1_state_sub,nei2_state_sub;
    // 支持2维激光雷达、3维激光雷达、D435i等实体传感器
    // 支持直接输入全局已知点云
    ros::Subscriber Gpointcloud_sub;
    ros::Subscriber Lpointcloud_sub;
    ros::Subscriber laserscan_sub;
    // ？

    // 发布控制指令
    ros::Publisher command_pub,path_cmd_pub;
    ros::Timer mainloop_timer, track_path_timer, safety_timer;

    // A星规划器
    Astar::Ptr Astar_ptr;

    prometheus_msgs::DroneState _DroneState;
    nav_msgs::Odometry Drone_odom;

    Eigen::Vector3d pos_nei[2];
    Eigen::Vector3d vel_nei[2];

    nav_msgs::Path path_cmd;
    double distance_walked;
    prometheus_msgs::SwarmCommand Command_Now;   

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
    Eigen::Vector3d start_pos, start_vel, goal_pos, goal_vel;

    float desired_yaw;

    ros::Time tra_start_time;
    float tra_running_time;
    
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

    // 回调函数
    void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg);
    void drone_state_cb(const prometheus_msgs::DroneStateConstPtr &msg);
    void nei_state_cb(const prometheus_msgs::DroneState::ConstPtr &msg, int nei_id);
    void Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void laser_cb(const sensor_msgs::LaserScanConstPtr &msg);

    void safety_cb(const ros::TimerEvent& e);
    void mainloop_cb(const ros::TimerEvent& e);
    void track_path_cb(const ros::TimerEvent& e);
   

    // 【获取当前时间函数】 单位：秒
    float get_time_in_sec(const ros::Time& begin_time);

    int get_start_point_id(void);
    
public:
    Swarm_Planner(void):
        swarm_planner_nh("~")
    {}~Swarm_Planner(){}

    void init(ros::NodeHandle& nh);
};

}

#endif