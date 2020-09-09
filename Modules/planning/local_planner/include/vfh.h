#ifndef VFH_H
#define VFH_H


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

class VFH: public local_planning_alg{
private:
    bool has_local_map_;
    bool is_prev;
    
    double goalWeight, prevWeight, headingWeight, obstacle_weight;

    double obs_distance;   // 感知距离
    // double min_dist;
    // double k_push;

    double max_att_dist;    // 最大吸引距离
    // double k_att;
    double ground_height;
    double ground_safe_height;
    double inflate_and_safe_distance;
    double inflate_distance;

    double safe_distance;
    double prev_heading;
    double limit_v_norm;

    double* Hdata;
    double  Hres;
    int Hcnt;  // 直方图个数
    // Eigen::Matrix<double, 3, 1> push_force;
    // Eigen::Matrix<double, 3, 1> attractive_force;

    pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;
    sensor_msgs::PointCloud2ConstPtr  local_map_ptr_;

    ros::Time begin_update_map;
    nav_msgs::Odometry cur_odom_;
    Eigen::Matrix<double, 3, 1>  cur_goal; 

    bool isIgnored(float x, float y, float z, float ws);
    int find_Hcnt(double angle);
    double find_angle(int cnt);
    double angle_error(double angle1, double angle2);

    void generate_voxel_data(double angle_cen, double angle_range, double val);

    int find_optimization_path(void);
public:
    int replan{0};
    
    virtual void set_odom(nav_msgs::Odometry cur_odom);
    
    virtual void set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr);
    virtual void set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr);
    
    virtual int compute_force(Eigen::Matrix<double, 3, 1> &goal, Eigen::Matrix<double, 3, 1> current_odom, Eigen::Vector3d &desired_vel);

    virtual void init(ros::NodeHandle& nh);

    VFH(){}
    ~VFH(){
        delete Hdata;
    }

    typedef shared_ptr<VFH> Ptr;

};

}

#endif 
