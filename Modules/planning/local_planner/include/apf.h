#ifndef APF_H
#define APF_H


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

#include "tools.h"
#include "local_planning_alg.h"
using namespace std;

namespace local_planner{

extern ros::Publisher message_pub;

class APF:public local_planning_alg{
private:
    bool has_local_map_;
    
    double obs_distance;
    double min_dist;
    double k_push;

    double max_att_dist;
    double k_att;
    double ground_height;
    double ground_safe_height;
    double inflate_distance;
    double safe_distance;

    Eigen::Matrix<double, 3, 1> push_force;
    Eigen::Matrix<double, 3, 1> attractive_force;

    pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;
    sensor_msgs::PointCloud2ConstPtr  local_map_ptr_;

    ros::Time begin_update_map;
    nav_msgs::Odometry cur_odom_;
    
public:
    int replan{0};
    virtual void set_odom(nav_msgs::Odometry cur_odom);
    virtual void set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr);
    virtual void set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr);
    virtual int compute_force(Eigen::Matrix<double, 3, 1> &goal, Eigen::Matrix<double, 3, 1> current_odom, Eigen::Vector3d &desired_vel);
    virtual void init(ros::NodeHandle& nh);
    APF(){}
    ~APF(){}

    typedef shared_ptr<APF> Ptr;

};




}

#endif 
