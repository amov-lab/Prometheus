/***************************************************************************************************************************
* pointcloude_to_octomap_node.cpp
*
* Author: Colin Lee
*
* Update Time: 2020.01.18
*
* Introduction:  Convert PointCloud to Octomap Node
*         1. 订阅点云的话题，作为构造八叉树地图的输入
*         2. 将八叉树地图转为ROS的消息，用话题发布
***************************************************************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明及定义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
const float kOctomapResolution(0.05f);
const std::string kCoordinateType("world");
// 构造八叉树
auto kOctoTree = std::make_shared<octomap::ColorOcTree>(kOctomapResolution);
ros::Publisher k_octomap_pub;
//【构造八叉树】 将点云的消息构造八叉树
void PointcloudToOctomap(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg) {
  if (pointCloudMsg->data.size()) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*pointCloudMsg, *globalCloud);

    kOctoTree->clear();
    for (auto p : globalCloud->points) {
      kOctoTree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
      kOctoTree->integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
    }
    kOctoTree->updateInnerOccupancy();
  } else {
    ROS_WARN("no point in cloud to octomap");
  }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cloud_to_octomap_callback(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg) {
  // 将点云转为八叉树
  PointcloudToOctomap(pointCloudMsg);
  // 构造八叉树消息并发布
  octomap_msgs::Octomap octomapMsg;
  octomap_msgs::fullMapToMsg(*kOctoTree, octomapMsg);
  octomapMsg.header.frame_id = kCoordinateType;
  octomapMsg.header.stamp = ros::Time::now();
  k_octomap_pub.publish(octomapMsg);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_to_octomap");
  ros::NodeHandle nh;
  // 订阅点云消息
  ros::Subscriber sub = nh.subscribe("pcl_output", 1, cloud_to_octomap_callback);
  k_octomap_pub = nh.advertise<octomap_msgs::Octomap>("prometheus/octomap_output", 1);
  ros::spin();

  return 0;
}
