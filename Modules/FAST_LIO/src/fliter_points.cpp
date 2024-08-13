#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
ros::Subscriber mid_sub;
ros::Publisher mid_pub;
double filter_range;
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
        if (point.z <= filter_range)
            continue;
        // 将点添加到输出点云
        out_pointcloud.points.push_back(point);
    }
    out_pointcloud.header.stamp = pointcloud.header.stamp;
    out_pointcloud.header.frame_id = pointcloud.header.frame_id;
    sensor_msgs::convertPointCloudToPointCloud2(out_pointcloud, pointcloud2);
    mid_pub.publish(pointcloud2);
}
int main(int argc,char** argv){
  ros::init(argc, argv, "fliter_points");
  ros::NodeHandle nh;

  int uav_id = 1;
  nh.param<double>("filter_range", filter_range, 0.5);
  nh.param<int>("uav_id", uav_id, 1);
  mid_sub = nh.subscribe<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/mid_point_cloud_centers", 10, cloudCallback);
  mid_pub = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/octomap_point_cloud_centers", 10);
  
  ros::spin();
  return 0;
}
