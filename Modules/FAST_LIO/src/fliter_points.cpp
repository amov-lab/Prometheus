#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>

ros::Subscriber mid_sub,odom_sub;
ros::Publisher mid_pub;
double filter_range_x,filter_range_y,filter_range_z;
float Odom_x,Odom_y,Odom_z; 
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud){
    sensor_msgs::PointCloud pointcloud,out_pointcloud;
    sensor_msgs::PointCloud2 pointcloud2;

    sensor_msgs::convertPointCloud2ToPointCloud(*input_cloud, pointcloud);
    //filter_range = 0.5;
    // 遍历点云中的所有点
    for (size_t i = 0; i < pointcloud.points.size(); ++i)
    {
        const auto& point = pointcloud.points[i];
        // 计算点与参考点之间的距离,如果点在指定范围内，则跳过该点
        if ((point.z - Odom_z) > 10 || point.z <= filter_range_z || (point.x - Odom_x) <= -filter_range_x || (point.x - Odom_x) > filter_range_x || (point.y - Odom_y) <= -filter_range_y || (point.y - Odom_y) > filter_range_y)
            continue;
        // 将点添加到输出点云
        out_pointcloud.points.push_back(point);
    }
    out_pointcloud.header.stamp = ros::Time::now();
    out_pointcloud.header.frame_id = input_cloud->header.frame_id;
    sensor_msgs::convertPointCloudToPointCloud2(out_pointcloud, pointcloud2);
    mid_pub.publish(pointcloud2);
}
void OdomCallback(const nav_msgs::Odometry::ConstPtr& input_odom){
    Odom_x = input_odom->pose.pose.position.x;
    Odom_y = input_odom->pose.pose.position.y;
    Odom_z = input_odom->pose.pose.position.z;
    //std::cout << "Odom_x: " << Odom_x << " Odom_y: " << Odom_y << " Odom_z: " << Odom_z << std::endl;
}
int main(int argc,char** argv){
  ros::init(argc, argv, "fliter_points");
  ros::NodeHandle nh;

  int uav_id = 1;
  nh.param<double>("filter_range_x", filter_range_x, 25.0);
  nh.param<double>("filter_range_y", filter_range_y, 25.0);
  nh.param<double>("filter_range_z", filter_range_z, 0.3);
  nh.param<int>("uav_id", uav_id, 1);
  mid_sub = nh.subscribe<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/mid360_world_point", 10, cloudCallback);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 10, OdomCallback);
  mid_pub = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/mid360_point_cloud_centers", 10);
  
  ros::spin();
  return 0;
}