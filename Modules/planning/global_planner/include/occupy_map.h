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
#include <nav_msgs/OccupancyGrid.h>

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
    // 地图是否占据容器， 从编程角度来讲，这就是地图变为单一序列化后的索引
    std::vector<int> occupancy_buffer_;  // 0 is free, 1 is occupied
    // 地图分辨率
    double resolution_, inv_resolution_;
    // 膨胀参数
    double inflate_;
    // 地图原点,地图尺寸
    Eigen::Vector3d origin_, map_size_3d_, min_range_, max_range_;
    // 占据图尺寸 = 地图尺寸 / 分辨率
    Eigen::Vector3i grid_size_;

    bool has_global_point;
   
    // 天花板高度,地板高度
    double ceil_height_, floor_height_;

    // 
    ros::Publisher inflate_cloud_pub_;
    
    Occupy_map(){}
    //初始化
    void init(void);

    // 设置参数
    void setparam(ros::NodeHandle& nh);
    // 设置全局点云指针
    void setEnvironment(const sensor_msgs::PointCloud2ConstPtr & global_point);
    // 地图膨胀
    void inflate_point_cloud(void);
    // 判断当前点是否在地图内
    bool isInMap(Eigen::Vector3d pos);
    // 设置占据
    void setOccupancy(Eigen::Vector3d pos, int occ);
    // 由位置计算索引
    void posToIndex(Eigen::Vector3d pos, Eigen::Vector3i &id);
    // 由索引计算位置
    void indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos);
    // 根据位置返回占据状态
    int getOccupancy(Eigen::Vector3d pos);
    // 根据索引返回占据状态
    int getOccupancy(Eigen::Vector3i id);
    // 检查安全
    bool check_safety(Eigen::Vector3d& pos, double check_distance/*, Eigen::Vector3d& map_point*/);

    typedef std::shared_ptr<Occupy_map> Ptr;
};

}



#endif