#ifndef APF_H
#define APF_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tools.h"
#include "local_planning_alg.h"
using namespace std;

namespace Local_Planning
{

extern ros::Publisher message_pub;

class APF:public local_planning_alg
{
private:
    //　参数
    double inflate_distance;
    double sensor_max_range;
    double max_att_dist;
    double k_push;
    double k_att;
    double min_dist;
    double ground_height;
    double ground_safe_height;
    double safe_distance;

    bool has_local_map_;
    bool has_odom_;
    bool is_2D;
    
    Eigen::Vector3d push_force;
    Eigen::Vector3d attractive_force;

    pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;
    sensor_msgs::PointCloud2ConstPtr  local_map_ptr_;
    nav_msgs::Odometry cur_odom_;
    
public:

    virtual void set_odom(nav_msgs::Odometry cur_odom);
    virtual void set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr);
    virtual void set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr);
    virtual int compute_force(Eigen::Vector3d &goal, Eigen::Vector3d &desired_vel);
    virtual void init(ros::NodeHandle& nh);
    APF(){}
    ~APF(){}

    typedef shared_ptr<APF> Ptr;

};




}

#endif 
