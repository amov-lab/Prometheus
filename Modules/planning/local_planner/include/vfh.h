#ifndef VFH_H
#define VFH_H

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

class VFH: public local_planning_alg
{
private:
    //　参数
    double inflate_distance;
    double sensor_max_range;
    double safe_distance;

    bool has_local_map_;
    bool has_odom_;
    bool is_2D;
    
    double goalWeight, obstacle_weight;

    double inflate_and_safe_distance;

    double limit_v_norm;

    double* Hdata;
    double  Hres;
    int Hcnt;  // 直方图个数

    pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;
    sensor_msgs::PointCloud2ConstPtr  local_map_ptr_;
    nav_msgs::Odometry cur_odom_;

    bool isIgnored(float x, float y, float z, float ws);
    int find_Hcnt(double angle);
    double find_angle(int cnt);
    double angle_error(double angle1, double angle2);
    void generate_voxel_data(double angle_cen, double angle_range, double val);
    int find_optimization_path(void);

public:

    virtual void set_odom(nav_msgs::Odometry cur_odom);
    virtual void set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr);
    virtual void set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr);
    virtual int compute_force(Eigen::Vector3d  &goal, Eigen::Vector3d &desired_vel);
    virtual void init(ros::NodeHandle& nh);

    VFH(){}
    ~VFH(){
        delete Hdata;
    }

    typedef shared_ptr<VFH> Ptr;

};

}

#endif 
