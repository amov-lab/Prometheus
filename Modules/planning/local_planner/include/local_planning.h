//
// Created by taojiang on 2019/12/9.
//

#ifndef SRC_POTENTIAL_FIELD_PLANNING_H
#define SRC_POTENTIAL_FIELD_PLANNING_H
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "prometheus_msgs/PositionReference.h"

#include "apf.h"

#include "planning_visualization.h"

using namespace std;

namespace local_planner{

class PotentialFiledPlanner{

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
    Eigen::Vector3d start_pt_, start_vel_,  end_pt_, end_vel_;
    pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;

    Eigen::Vector3d desired_vel;

    sensor_msgs::PointCloud2ConstPtr  local_map_ptr_, global_map_ptr_;

    void generate_cmd(Eigen::Vector3d desired_vel);

    void getOccupancyMarker(visualization_msgs::Marker &m, int id, Eigen::Vector4d color);

    void localframe2global(void){
        // 将局部坐标系点云，转化到全局坐标系,
        // 因为一般给定了目标点是全局坐标系（地图系）下， 同一在地图系下进行计算
    }
  
    void waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void globalcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void localcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void execFSMCallback(const ros::TimerEvent& e);

    // 控制接口
    prometheus_msgs::PositionReference cmd;
    geometry_msgs::Point px4_cmd;

    /* ---------- sub and pub ---------- */
    ros::NodeHandle node_;
    ros::Subscriber waypoint_sub_, odom_sub_, global_point_clound_sub_, local_point_clound_sub_;
    ros::Timer exec_timer_;
    ros::Time control_time;

    ros::Publisher pos_cmd_pub, local_map_marker_Pub, px4_pos_cmd_pub;

    APF::Ptr apf_planner_ptr;

    PlanningVisualization::Ptr visualization_;
    int is_simulation;
public:

    nav_msgs::Odometry odom_, odom__last;

    double obs_distance;
    double att_distance;

    Eigen::Matrix<double, 3, 1> total_force;

    PotentialFiledPlanner(/* args */):node_("~") {
        
    }

    ~PotentialFiledPlanner(){}
    void init(ros::NodeHandle& nh);

    /* get */

    /* set */


    /* check feasibility*/


    /* for evaluation */


    // typedef std::shared_ptr<PotentialFiledPlanner> Ptr;
};



}



#endif //SRC_POTENTIAL_FIELD_PLANNING_H
