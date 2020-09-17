//
// Created by taojiang on 2019/12/9.
//

#ifndef SRC_POTENTIAL_FIELD_PLANNING_H
#define SRC_POTENTIAL_FIELD_PLANNING_H
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "prometheus_msgs/Message.h"
#include <std_msgs/Bool.h>


#include "apf.h"
#include "local_planning.h"
#include "vfh.h"

#include "planning_visualization.h"

#include "tools.h"

#include "message_utils.h"

using namespace std;
#define NODE_NAME "local_planner"

namespace local_planner{

extern ros::Publisher message_pub;

class LocalPlanningClass{

private:

  /* ---------- flag ---------- */
    enum EXEC_STATE
    {
        INIT,
        WAIT_GOAL,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ
    };
    EXEC_STATE exec_state_;

    enum FLIGHT_TYPE
    {
        MANUAL_GOAL = 1,
        PRESET_GOAL = 2,
        INPUT_MANUAL
    };

    bool trigger_, have_goal_, has_point_map_;
    bool have_odom_;
    int flight_type_;
    int lidar_model;
    double max_planning_vel;
    Eigen::Vector3d start_pt_, start_vel_,  end_pt_, end_vel_;
    pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;
    std_msgs::Int8 replan;

    Eigen::Vector3d desired_vel;

    sensor_msgs::PointCloud2ConstPtr  local_map_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;

    void generate_cmd(Eigen::Vector3d desired_vel);

    void getOccupancyMarker(visualization_msgs::Marker &m, int id, Eigen::Vector4d color);
  
    void waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void localcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void execFSMCallback(const ros::TimerEvent& e);
    void switchCallback(const std_msgs::Bool::ConstPtr &msg);
    // 控制接口
    geometry_msgs::Point px4_cmd;

    /* ---------- sub and pub ---------- */
    ros::NodeHandle node_;
    ros::Subscriber waypoint_sub_, odom_sub_, global_point_clound_sub_, local_point_clound_sub_, swith_sub;
    ros::Timer exec_timer_;
    ros::Time control_time;

    ros::Publisher local_map_marker_Pub, px4_pos_cmd_pub;
    ros::Publisher replan_cmd_Pub;

    // 局部避障算法 算子
    local_planning_alg::Ptr local_alg_ptr;

    PlanningVisualization::Ptr visualization_;
    int algorithm_mode; // 0  is apf; 1 is vfh (default is 0)

public:

    nav_msgs::Odometry odom_, odom__last;

    double obs_distance;
    double att_distance;

    Eigen::Matrix<double, 3, 1> total_force;

    LocalPlanningClass(/* args */):node_("~") {
    }

    ~LocalPlanningClass(){
    }
    void init(ros::NodeHandle& nh);

    /* get */

    /* set */


    /* check feasibility*/


    /* for evaluation */


    // typedef std::shared_ptr<LocalPlanningClass> Ptr;
};



}



#endif //SRC_POTENTIAL_FIELD_PLANNING_H
