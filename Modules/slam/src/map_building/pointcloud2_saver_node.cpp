#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>  
  
void cloudCB(const sensor_msgs::PointCloud2 &input)  
{  
  pcl::PointCloud<pcl::PointXYZ> cloud;  
  pcl::fromROSMsg(input, cloud);
  pcl::io::savePCDFileASCII ("~/Desktop/test.pcd", cloud);  
}  

main (int argc, char **argv)  
{  
  ros::init (argc, argv, "pcl_write");  
  ros::NodeHandle nh;  
  ros::Subscriber bat_sub = nh.subscribe("/octomap_point_cloud_centers", 10, cloudCB);//接收点云  
  ros::spin();  
  return 0;  
} 
