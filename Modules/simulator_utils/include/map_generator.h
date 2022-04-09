#ifndef MAP_GENERATOR_H
#define MAP_GENERATOR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <iostream>
#include <Eigen/Eigen>
#include <random>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <prometheus_msgs/UAVState.h>

#include "printf_utils.h"

using namespace std;
using namespace Eigen;

#define MAX_SWRAM_NUM 41

// 节点功能：生成随机地图，并模拟无人机检测，模拟得到局部地图与全局地图
class Map_Generator
{
public:
    Map_Generator(){};
    void init(ros::NodeHandle &nh);

    int swarm_num;                                     // 集群数量
    int uav_id[MAX_SWRAM_NUM];                         // 编号
    double map_size_x, map_size_y, map_size_z;         // 地图尺寸
    double map_x_min, map_x_max, map_y_min, map_y_max; // 地图xy范围
    double map_z_limit;
    double map_resolution;              // 地图分辨率
    double sensing_range, sensing_rate; // 传感器感知半径与频率
    double sensing_horizon;

    double obs_min_dist;                     // 障碍物最小间距
    int cylinder_num;                        // 圆柱体数量
    double cylinder_radius, cylinder_height; // 圆柱体参数
    int cuboid_num;                          // 立方体数量
    double cuboid_size, cuboid_height;       // 立方体参数
    double wall_length, wall_height;         // 墙参数
    double line_height;
    // 初始状态量
    
    bool uav_odom_ok[MAX_SWRAM_NUM] = {false};
    // 无人机初始位置
    double uav_init_x, uav_init_y;
    nav_msgs::Odometry uav_odom[MAX_SWRAM_NUM];

    bool global_map_ok = false;
    pcl::PointCloud<pcl::PointXYZ> global_map_pcl; // 全局点云地图 - pcl格式
    sensor_msgs::PointCloud2 global_map_ros;       // 全局点云地图 - ros_msg格式
    pcl::PointCloud<pcl::PointXYZ> local_map_pcl;  // 局部点云地图 - pcl格式
    sensor_msgs::PointCloud2 local_map_ros;        // 局部点云地图 - ros_msg格式
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;

    // 基本形状生成函数
    void generate_cylinder(double x, double y);
    void generate_cuboid(double x, double y);
    void generate_row_wall(double x, double y);
    void generate_column_wall(double x, double y);
    void generate_line(double x, double y);

    // 边界生成
    void GenerateBorder();
    // 随机地图生成
    void GenerateRandomMap();
    // 固定地图生成
    void GenerateMap1();

private:
    // 订阅
    ros::Subscriber uav_odom_sub[MAX_SWRAM_NUM];
    // 发布
    ros::Publisher local_map_pub[MAX_SWRAM_NUM];
    ros::Publisher global_map_pub;
    // 定时器
    ros::Timer pub_local_map_timer[MAX_SWRAM_NUM];
    ros::Timer pub_global_map_timer;

    random_device rd;
    default_random_engine eng;
    uniform_real_distribution<double> rand_x;
    uniform_real_distribution<double> rand_y;

    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;

    void uav_odom_cb(const nav_msgs::Odometry::ConstPtr &odom, int uav_id);
    void pub_local_map_cb(const ros::TimerEvent &event, int uav_id);
    void pub_global_map_cb(const ros::TimerEvent &event);
};

#endif