#ifndef SRC_GLOBAL_PLANNING
#define SRC_GLOBAL_PLANNING


#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/Message.h"
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"

#include "planning_visualization.h"
#include <tf/transform_listener.h>
#include "A_star.h"
#include "tools.h"
#include "message_utils.h"
using namespace std;

#define NODE_NAME "global_planner"

namespace global_planner{

extern ros::Publisher message_pub;

class GlobalPlanner{

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

    int flight_type_;

    // state
    bool have_odom_;
    bool has_point_map_;
    bool trigger_; // waypoint 
    bool have_goal_; 
    std_msgs::Int8 replan;
    double safe_distance;
    int is_2D; // 1代表2D平面规划及搜索,0代表3D
    double fly_height;

    nav_msgs::Odometry odom_, odom__last;
    nav_msgs::Path A_star_path_cmd;
    pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;
    pcl::PointCloud<pcl::PointXYZ> latest_global_pcl_;
    sensor_msgs::PointCloud2ConstPtr  local_map_ptr_, global_map_ptr_;

    Eigen::Vector3d start_pt_, start_vel_,  end_pt_, end_vel_;

    /* ---------- sub and pub ---------- */
    void waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void globalcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void safetyCallback(const ros::TimerEvent& e);
    void execCallback(const ros::TimerEvent& e);
    void switchCallback(const std_msgs::Bool::ConstPtr &msg);

    ros::NodeHandle node_;
    ros::Subscriber waypoint_sub_, odom_sub_, global_point_clound_sub_, local_point_clound_sub_, swith_sub;
    ros::Timer exec_timer_, safety_timer_;
    ros::Time control_time;

    ros::Publisher pos_cmd_pub, global_map_marker_Pub, px4_pos_cmd_pub;
    ros::Publisher path_cmd_Pub;
    ros::Publisher replan_cmd_Pub;

    // visual 
    PlanningVisualization::Ptr visualization_;

    Astar::Ptr Astar_ptr;
    void generate_CMD(std::vector<Eigen::Vector3d> path);
    
public:
    
    void getOccupancyMarker(visualization_msgs::Marker &m, int id, Eigen::Vector4d color);

    GlobalPlanner(/* args */):node_("~") {
        
    }
    ~GlobalPlanner(){}

    void init(ros::NodeHandle& nh);
    
    /* get */

    /* set */


    /* check feasibility*/


    /* for evaluation */


    // typedef std::shared_ptr<GlobalPlanner> Ptr;

};

}

#endif //SRC_GLOBAL_PLANNING