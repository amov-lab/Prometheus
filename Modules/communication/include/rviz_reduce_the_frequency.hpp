#ifndef REDUCE_THE_FREQUENCY_HPP
#define REDUCE_THE_FREQUENCY_HPP

#include "sensor_msgs/PointCloud2.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"

#include <ros/serialization.h>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

class ReduceTheFrequency
{
public:
    ReduceTheFrequency(ros::NodeHandle &nh);
    ~ReduceTheFrequency();

private:
    void octomapPointCloudCentersCb(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void occupancyInflateCb(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void scanCb(const sensor_msgs::LaserScan::ConstPtr &msg);
    void scanFilteredCb(const sensor_msgs::LaserScan::ConstPtr &msg);

    void trajectoryCb(const nav_msgs::Path::ConstPtr &msg);

    void odomCb(const nav_msgs::Odometry::ConstPtr &msg);

    void tfCb(const tf2_msgs::TFMessage::ConstPtr &msg);

    void UAVMeshCb(const visualization_msgs::Marker::ConstPtr &msg);

    void optimalListCb(const visualization_msgs::Marker::ConstPtr &msg);

    void send1000MS(const ros::TimerEvent &time_event);

    void send500MS(const ros::TimerEvent &time_event);

    void send200MS(const ros::TimerEvent &time_event);

    void send50MS(const ros::TimerEvent &time_event);

    sensor_msgs::PointCloud2 filtered(const sensor_msgs::PointCloud2 msg);

    sensor_msgs::PointCloud2 compressed(const sensor_msgs::PointCloud2 msg);

private:
    ros::Subscriber octomap_point_cloud_centers_sub_,occupancy_inflate_sub_,scan_sub_,
        scan_filtered_sub_,trajectory_sub_,odom_sub_,tf_sub_,uav_mesh_sub_,optimal_list_sub_;

    // 1000ms
    ros::Publisher octomap_pub_,occupancy_inflate_pub_,scan_pub_,scan_filtered_pub_,
        occupancy_inflate_filtered_pub_,occupancy_inflate_compressed_pub_,octomap_compressed_pub_;
    // 500ms
    ros::Publisher trajectory_pub_,uav_mesh_pub_;
    // 200ms
    ros::Publisher odom_pub_,tf_pub_,optimal_list_pub_;

    double uav_position_x = 0,uav_position_y = 0;

    bool octomap_point_cloud_ready = false;
    bool occupancy_inflate_ready = false;
    bool scan_ready = false;
    bool scan_filtered_ready = false;
    bool trajectory_ready = false;
    bool odom_ready = false;
    bool uav_mesh_ready = false;
    bool optimal_list_ready = false;

    sensor_msgs::PointCloud2 octomap_point_cloud,octomap_compressed_point_cloud;
    sensor_msgs::PointCloud2 occupancy_inflate_point_cloud,occupancy_inflate_filtered_point_cloud,occupancy_inflate_compressed_point_cloud;
    sensor_msgs::LaserScan scan;
    sensor_msgs::LaserScan scan_filtered;
    nav_msgs::Path trajectory;
    nav_msgs::Odometry odom;
    tf2_msgs::TFMessage tf;
    visualization_msgs::Marker uav_mesh;
    visualization_msgs::Marker optimal_list;

    ros::Timer send_timer_1000MS,send_timer_200MS,send_timer_500MS,send_timer_50MS;
};

#endif