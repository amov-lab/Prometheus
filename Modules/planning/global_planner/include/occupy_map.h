#ifndef _OCCUPY_MAP_H
#define _OCCUPY_MAP_H

#include <iostream>
#include <algorithm>

// #include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>


namespace global_planner
{
class Occupy_map{
public:
    sensor_msgs::PointCloud2ConstPtr global_env_;
    std::vector<int> occupancy_buffer_;  // 0 is free, 1 is occupied
    double resolution_, inv_resolution_;
    double inflate_;
    Eigen::Vector3d origin_, map_size_3d_, min_range_, max_range_;
    Eigen::Vector3i grid_size_;
    bool has_global_point;
   
    double ceil_height_, floor_height_;

    ros::Publisher inflate_cloud_pub_;

   Occupy_map(){}

   void init(void);

    void setparam(ros::NodeHandle& nh);

    void setEnvironment(const sensor_msgs::PointCloud2ConstPtr & global_point);

    void inflate_point_cloud(void);

    bool isInMap(Eigen::Vector3d pos) ;
    void setOccupancy(Eigen::Vector3d pos, int occ) ;
    void posToIndex(Eigen::Vector3d pos, Eigen::Vector3i &id);
    void indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos);
    int getOccupancy(Eigen::Vector3d pos) ;
    int getOccupancy(Eigen::Vector3i id);
    bool check_safety(Eigen::Vector3d& pos, double check_distance/*, Eigen::Vector3d& map_point*/);
    typedef std::shared_ptr<Occupy_map> Ptr;
};

}



#endif