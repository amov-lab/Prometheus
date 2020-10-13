#ifndef _OCCUPY_MAP_H
#define _OCCUPY_MAP_H

#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <Eigen/Eigen>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

#include "tools.h"
#include "message_utils.h"
#define NODE_NAME "global_planner"
namespace global_planner
{

extern ros::Publisher message_pub;

class Occupy_map{
public:
    // 全局点云指针
    sensor_msgs::PointCloud2ConstPtr global_env_;
    // 容器
    std::vector<int> occupancy_buffer_;  // 0 is free, 1 is occupied
    // 地图分辨率
    double resolution_, inv_resolution_;
    // 膨胀参数
    double inflate_;
    // 地图原点,地图尺寸
    Eigen::Vector3d origin_, map_size_3d_, min_range_, max_range_;
    // 
    Eigen::Vector3i grid_size_;

    bool has_global_point;
   
    // 天花板高度,地板高度
    double ceil_height_, floor_height_;

    // 
    ros::Publisher inflate_cloud_pub_;
    
    Occupy_map(){}
    void init(void);

    void setparam(ros::NodeHandle& nh);

    // 设置地图环境
    void setEnvironment(const sensor_msgs::PointCloud2ConstPtr & global_point);
    // 地图膨胀
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