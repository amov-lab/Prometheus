/**
 * @file pointcloud_to_octomap_node.cpp
 * @brief this file is a ros node subscribing pcl_output then convert to octomap and publish
 * @date 2019-02-25 edit by lfy
 * @copyright Copyright (c) 2019
 */
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

const float kOctomapResolution(0.05f);
const std::string kCoordinateType("world");

auto kOctoTree = std::make_shared<octomap::ColorOcTree>(kOctomapResolution);
ros::Publisher k_octomap_pub;

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

/**
 * @brief this function convert pcl to octomap and publish octomap
 */
void cloud_to_octomap_callback(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg) {
  PointcloudToOctomap(pointCloudMsg);

  octomap_msgs::Octomap octomapMsg;
  octomap_msgs::fullMapToMsg(*kOctoTree, octomapMsg);
  octomapMsg.header.frame_id = kCoordinateType;
  octomapMsg.header.stamp = ros::Time::now();
  k_octomap_pub.publish(octomapMsg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_to_octomap");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("pcl_output", 1, cloud_to_octomap_callback);
  k_octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap_output", 1);
  ros::spin();

  return 0;
}
