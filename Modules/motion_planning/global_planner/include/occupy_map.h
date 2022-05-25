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
#include "global_planner_utils.h"
using namespace std;

namespace GlobalPlannerNS
{
    class Occupy_map
    {
    public:
        Occupy_map(){}
        double fly_height;

        // 局部地图 滑窗 存储器
        map<int, pcl::PointCloud<pcl::PointXYZ>> point_cloud_pair;
        // 全局地图点云指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_point_cloud_map;
        // 全局膨胀点云指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_uav_pcl;
        // 考虑变指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inflate_vis_;
        // 临时指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
        // 地图边界点云
        pcl::PointCloud<pcl::PointXYZ> border;
        // VoxelGrid过滤器用于下采样
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        // OutlierRemoval用于去除离群值
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // laserscan2pointcloud2 中间变量
        sensor_msgs::PointCloud2 input_laser_scan;
        // laserscan2pointcloud2 投影器
        laser_geometry::LaserProjection projector_;
        // 上一帧odom
        double f_x, f_y, f_z, f_roll, f_pitch, f_yaw;
        // 局部地图滑窗，指示器以及大小
        int st_it, queue_size;
        // flag：展示地图边界
        bool show_border;
        bool sim_mode;
        // 地图是否占据容器， 从编程角度来讲，这就是地图变为单一序列化后的索引
        std::vector<int> occupancy_buffer_; // 0 is free, 1 is occupied
        // 代价地图
        std::vector<double> cost_map_; // cost
        // 地图分辨率
        double resolution_, inv_resolution_;
        string node_name;
        // 膨胀参数
        double inflate_;
        double odom_inflate_;
        // 地图原点,地图尺寸
        Eigen::Vector3d origin_, map_size_3d_, min_range_, max_range_;
        // 占据图尺寸 = 地图尺寸 / 分辨率
        Eigen::Vector3i grid_size_;
        int swarm_num_uav; // 集群数量
        string uav_name;   // 无人机名字
        int uav_id;        // 无人机编号
        bool get_gpcl, get_lpcl, get_laser;
        Eigen::Vector3d enum_p[100], enum_p_uav[1000], enum_p_cost[1000];
        int ifn;
        int inflate_index, inflate_index_uav, cost_index, cost_inflate;
        // 发布点云用于rviz显示
        ros::Publisher global_pcl_pub, inflate_pcl_pub;
        ros::Timer pcl_pub;

        //初始化
        void init(ros::NodeHandle &nh);
        // 地图更新函数 - 输入：全局点云
        void map_update_gpcl(const sensor_msgs::PointCloud2ConstPtr &global_point);
        // 工具函数：合并局部地图
        void local_map_merge_odom(const nav_msgs::Odometry &odom);
        void uav_pcl_update(Eigen::Vector3d *input_uav_odom, bool *get_uav_odom);
        // 地图更新函数 - 输入：局部点云
        void map_update_lpcl(const sensor_msgs::PointCloud2ConstPtr &local_point, const nav_msgs::Odometry &odom);
        // 地图更新函数 - 输入：二维激光雷达
        void map_update_laser(const sensor_msgs::LaserScanConstPtr &local_point, const nav_msgs::Odometry &odom);
        // 地图膨胀
        void inflate_point_cloud(void);
        // 判断当前点是否在地图内
        bool isInMap(Eigen::Vector3d pos);
        // 设置占据
        void setOccupancy(Eigen::Vector3d &pos, int occ);
        // 设置代价
        void updateCostMap(Eigen::Vector3d &pos, double cost);
        // 由位置计算索引
        void posToIndex(Eigen::Vector3d &pos, Eigen::Vector3i &id);
        // 由索引计算位置
        void indexToPos(Eigen::Vector3i &id, Eigen::Vector3d &pos);
        // 根据位置返回占据状态
        int getOccupancy(Eigen::Vector3d &pos);
        // 根据索引返回占据状态
        int getOccupancy(Eigen::Vector3i &id);
        // 根据索引返回代价
        double getCost(Eigen::Vector3d &pos);
        // 检查安全
        bool check_safety(Eigen::Vector3d &pos, double check_distance /*, Eigen::Vector3d& map_point*/);
        void pub_pcl_cb(const ros::TimerEvent &e);
        // 定义该类的指针
        typedef std::shared_ptr<Occupy_map> Ptr;
    };

} // namespace GlobalPlannerNS

#endif