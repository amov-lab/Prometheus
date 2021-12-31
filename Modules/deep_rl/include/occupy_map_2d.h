#ifndef _OCCUPY_MAP_H
#define _OCCUPY_MAP_H

#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <map>
#include "printf_utils.h"
using namespace std;

namespace drl_ns
{

class Occupy_map
{
    public:
        Occupy_map(){}

        // 发布点云用于rviz显示
        ros::Publisher global_pcl_pub;
        ros::Publisher global_inflate_pcl_pub;
        ros::Timer map_pub_timer;

        int ugv_id;                                     // 无人机编号
        string ugv_name;                                // 无人机名字
        // 地图分辨率
        double resolution_, inv_resolution_;
        // 膨胀参数
        double inflate_;
        double odom_inflate_;
        // 地图原点,地图尺寸
        Eigen::Vector3d origin_, map_size_3d_, min_range_, max_range_;
        // 占据图尺寸 = 地图尺寸 / 分辨率
        Eigen::Vector3i grid_size_;
        // flag：展示地图边界
        bool show_border;
        int ifn;
        int inflate_index, inflate_index_ugv, cost_index, cost_inflate;
        double ugv_height;
        // 上一帧odom
        double f_x, f_y, f_z, f_roll, f_pitch, f_yaw;
        // 局部地图滑窗，指示器以及大小
        int st_it, queue_size;
        // 地图是否占据容器， 从编程角度来讲，这就是地图变为单一序列化后的索引
        std::vector<int> occupancy_buffer_;  // 0 is free, 1 is occupied

        // 全局点云指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_pcl_ptr;
        // 全局膨胀点云指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_inflate_pcl_ptr;
        // 其他无人车点云指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr other_ugv_pcl_ptr;
        // 输入点云指针 - 临时
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl_ptr;
        // 输入点云指针（坐标转换后） - 临时
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_transformed_pcl_ptr;
        // 局部地图 滑窗 存储器
        map<int,pcl::PointCloud<pcl::PointXYZ>> point_cloud_pair;

        // 地图边界点云
        pcl::PointCloud<pcl::PointXYZ> border;
        // VoxelGrid过滤器用于下采样
        pcl::VoxelGrid<pcl::PointXYZ> vg; 

        Eigen::Vector3d enum_p[100], enum_p_ugv[1000], enum_p_cost[1000];

        //初始化
        void init(ros::NodeHandle& nh, int id);
        void reset();
        // 定时器回调
        void map_pub_cb(const ros::TimerEvent& e);
        // 地图更新函数 - 输入：二维激光雷达
        void map_update_laser(const sensor_msgs::LaserScanConstPtr & local_point, const nav_msgs::Odometry & odom);
        // 地图更新函数
        void map_update_other_ugv(Eigen::Vector3d *input_ugv_odom, bool *get_ugv_odom, int swarm_num);
        // 工具函数：合并局部地图
        void local_map_merge_odom(const nav_msgs::Odometry & odom);
        // 地图膨胀
        void inflate_point_cloud(void);
        // 设置占据
        void setOccupancy(Eigen::Vector3d &pos, int occ);
        // 判断当前点是否在地图内
        bool isInMap(Eigen::Vector3d pos);
        // 由位置计算索引
        void posToIndex(Eigen::Vector3d &pos, Eigen::Vector3i &id);
        // 由索引计算位置
        void indexToPos(Eigen::Vector3i &id, Eigen::Vector3d &pos);
        // 根据位置返回占据状态
        int getOccupancy(Eigen::Vector3d &pos);
        // 根据索引返回占据状态
        int getOccupancy(Eigen::Vector3i &id);
        int getOccupancy(Eigen::Vector3d &pos, float distance);
        // 检查安全
        bool check_safety(Eigen::Vector3d &pos, double check_distance/*, Eigen::Vector3d& map_point*/);
        
        // 定义该类的指针
        typedef std::shared_ptr<Occupy_map> Ptr;
};

}

#endif