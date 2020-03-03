/***************************************************************************************************************************
* pointcloude_to_octomap_node.cpp
*
* Author: Colin Lee
*
* Update Time: 2020.02.29
*
* Introduction:  Convert Depth Map to Octomap Node
*         1. 订阅深度图和位姿的话题，构造点云
*         2. 根据点云构造八叉树地图
*         2. 将八叉树地图转为ROS的消息，用话题发布
***************************************************************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明及定义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
const float kOctomapResolution(0.05f);
const std::string kCoordinateType("world");
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud *pointCloud = new PointCloud;
sensor_msgs::PointCloud2 output;
// 构造八叉树
auto kOctoTree = std::make_shared<octomap::ColorOcTree>(kOctomapResolution);
ros::Publisher k_octomap_pub;
ros::Publisher pcl_pub;
//【构造八叉树】 将点云的消息构造八叉树
void PointcloudToOctomap(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg)
{
  if (pointCloudMsg->data.size())
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*pointCloudMsg, *globalCloud);

    kOctoTree->clear();
    for (auto p : globalCloud->points)
    {
      kOctoTree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
      kOctoTree->integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
    }
    kOctoTree->updateInnerOccupancy();
  }
  else
  {
    ROS_WARN("no point in cloud to octomap");
  }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cloud_to_octomap_callback(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg)
{
  // 将点云转为八叉树
  PointcloudToOctomap(pointCloudMsg);
  // 构造八叉树消息并发布
  octomap_msgs::Octomap octomapMsg;
  octomap_msgs::fullMapToMsg(*kOctoTree, octomapMsg);
  octomapMsg.header.frame_id = kCoordinateType;
  octomapMsg.header.stamp = ros::Time::now();
  k_octomap_pub.publish(octomapMsg);
}

void rgbd_mapping_callback(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImageConstPtr &depth)
{
  // Transfer quaternion into matrix
  double x, y, z, qx, qy, qz, qw;
  x = odom->pose.pose.position.x;
  y = odom->pose.pose.position.y;
  z = odom->pose.pose.position.z;
  qx = odom->pose.pose.orientation.x;
  qy = odom->pose.pose.orientation.y;
  qz = odom->pose.pose.orientation.z;
  qw = odom->pose.pose.orientation.w;
  Eigen::Quaterniond q(qw, qx, qy, qz);
  Eigen::Isometry3d Trans(q);
  Trans.pretranslate(Eigen::Vector3d(x, y, z));
  // Make global pointcloud
  cv::Mat depthImage;
  depthImage = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1)->image;
  cv::imshow("depth", depthImage);
  cv::waitKey(5);
  int width = depthImage.cols;
  int height = depthImage.rows;
  //ROS_INFO("%f", depthImage.at<float>(height / 2, width / 2));
  double fx = 381.05621337890625;
  double fy = 381.05621337890625;
  double cx = 318.6089782714844;
  double cy = 233.97291564941406;
  for (int v = 0; v < height; v += 10)
    for (int u = 0; u < width; u += 10)
    {
      double d = static_cast<double>(depthImage.at<float>(v, u));
      Eigen::Vector3d point;
      point[2] = d;
      // std::cout << "the depth:" << d << std::endl;
      point[1] = (v - cy) * point[2] / fy;
      point[0] = (u - cx) * point[2] / fx;
      // point = rotation_vector.matrix()*point;
      Eigen::Vector3d pointWorld = Trans * point;

      PointT p;
      p.x = pointWorld[0];
      p.y = pointWorld[1];
      p.z = pointWorld[2];
      p.r = 255;
      p.g = 255;
      p.b = 255;
      pointCloud->points.push_back(p);
    }
  pointCloud->is_dense = false;
  pcl::toROSMsg(*pointCloud, output);
  output.header.frame_id = "map";
  pcl_pub.publish(output);
  pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping_by_rgbd");
  ros::NodeHandle nh;
  std::string odom_topic = "/t265/odom/sample";
  std::string depth_topic = "/d400/depth/image_rect_raw";

  // 订阅点云消息
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, odom_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 1);
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, depth_sub);

  sync.registerCallback(boost::bind(&rgbd_mapping_callback, _1, _2));
  //k_octomap_pub = nh.advertise<octomap_msgs::Octomap>("prometheus/octomap_output", 1);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("prometheus/pointcloud", 1);
  ros::spin();

  return 0;
}
