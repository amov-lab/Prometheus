#ifndef GLOBAL_PLANNER_UGV
#define GLOBAL_PLANNER_UGV

#include <ros/ros.h>
#include <boost/format.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/UGVState.h"
#include "prometheus_msgs/UGVCommand.h"
#include "prometheus_msgs/StationCommand.h"

#include "A_star.h"
#include "occupy_map.h"
#include "printf_utils.h"

#include "angles/angles.h"
#include <tf2/utils.h> // getYaw

using namespace std;

#define MIN_DIS 0.1

namespace global_planner_ugv
{

class GlobalPlannerUGV
{

private:
    // ros nh
    ros::NodeHandle global_planner_nh;
    // 【订阅】地面站指令
    ros::Subscriber station_cmd_sub;
    // 【订阅】目标点（手动模式）
    ros::Subscriber goal_sub;
    // 【订阅】无人车状态
    ros::Subscriber ugv_state_sub;
    // 【订阅】其他无人车odom
    ros::Subscriber nei_odom_sub[21];
    // 【订阅】传感器数据 - 全局点云、局部点云、Scan
    ros::Subscriber Gpointcloud_sub;
    ros::Subscriber Lpointcloud_sub;
    ros::Subscriber laserscan_sub;
    // 【发布】控制指令
    ros::Publisher command_pub;
    // 【发布】规划路径
    ros::Publisher path_cmd_pub;
    // 【定时器】主循环定时器、路径追踪定时器、目标追踪定时器
    ros::Timer mainloop_timer, track_path_timer, send_nei_odom_timer;
    // 五种状态机
    enum EXEC_STATE
    {
      INIT,
      WAIT_GOAL,
      PLAN,
      PATH_TRACKING,
      RETURN,
      STOP
    };
    EXEC_STATE exec_state;
    // 集群数量
    int swarm_num_ugv;     
    // 无人车名字                             
    string ugv_name;    
    float k_p,k_aoivd; 
    // 是否仿真模式
    bool sim_mode;    
    // 无人车编号                         
    int ugv_id;                                     
    // 无人车高度
    double ugv_height;
    double depth;
    double angle_y;
    // 手动给定目标点模式 或 自动目标点模式
    bool manual_mode;
    // 传感器输入flag
    int map_input_source;
    // 路径重规划时间
    double replan_time;
    double track_frequency;
    int counter_search;
    // 其他无人车位置
    nav_msgs::Odometry nei_odom[21];
    Eigen::Vector3d odom_nei[21];
    bool get_nei_odom[21];
    // A星规划器
    Astar::Ptr Astar_ptr;
    // A星规划器状态
    int astar_state;
    // 无人车状态
    prometheus_msgs::UGVState ugv_state;
    // 地面站指令
    prometheus_msgs::StationCommand station_cmd;
    // 发布的控制指令
    prometheus_msgs::UGVCommand Command_Now;
    // 无人车里程计信息
    nav_msgs::Odometry ugv_odom;
    //yaw
    float ugv_yaw;
    // 规划得到的路径
    nav_msgs::Path path_cmd;
    // 路经点开始id
    int start_point_index;
    // 路经点总数
    int Num_total_wp;
    // 当前执行ID
    int cur_id;
    // 距离上一次重置，移动的距离（重规划路径时重置）
    float distance_walked;
    float yaw_ref;
    // 上一次重置时，无人车的位置
    Eigen::Vector3d ugv_pos_last;
    // 规划初始状态及终端状态
    Eigen::Vector3d start_pos, start_vel, start_acc, goal_pos, goal_vel;
    // 返航位置
    Eigen::Vector3d return_pos;
    // 规划器状态
    bool odom_ready;
    bool ugv_ready;
    bool sensor_ready;
    bool station_ready;
    bool get_goal; 
    bool in_return_mode;
    bool path_ok;
    bool rotate_in_place;  // vinson add
    bool start_move; // vinson add
    bool get_target_pos;
    // 目标位置
    geometry_msgs::PoseStamped target_pos;
    // 上一条轨迹开始时间
    ros::Time tra_start_time;    
    
    // 回调函数
    void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg);
    void ugv_state_cb(const prometheus_msgs::UGVStateConstPtr &msg);
    void Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void laser_cb(const sensor_msgs::LaserScanConstPtr &msg);
    void cmd_cb(const prometheus_msgs::StationCommandConstPtr& msg);
    void nei_odom_cb(const nav_msgs::Odometry::ConstPtr& odom, int id);

    void send_nei_odom_cb(const ros::TimerEvent& e);
    void mainloop_cb(const ros::TimerEvent& e);
    void track_path_cb(const ros::TimerEvent& e);
   
    // 【获取当前时间函数】 单位：秒
    float get_time_in_sec(const ros::Time& begin_time);
    int get_start_point_id(void);
    void printf_exec_state();

    // vinson
    // tool function
    const int get_track_point_id();

    
public:
    GlobalPlannerUGV(/* args */)
    {    
    }
    ~GlobalPlannerUGV()
    {
    }

    void init(ros::NodeHandle& nh);
};

}

#endif
