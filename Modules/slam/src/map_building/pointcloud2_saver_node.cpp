#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>  
using namespace std;
string pcd_savepath;
void cloudCB(const sensor_msgs::PointCloud2 &input)  
{  
  pcl::PointCloud<pcl::PointXYZ> cloud;  
  pcl::fromROSMsg(input, cloud);
  pcl::io::savePCDFileASCII (pcd_savepath, cloud);  
}  

main (int argc, char **argv)  
{  
  ros::init (argc, argv, "pcl_write");  
  ros::NodeHandle nh("~"); 
  nh.param<string>("pcd_savepath", pcd_savepath, "~/Desktop/test.pcd");
  ros::Subscriber bat_sub = nh.subscribe("/octomap_point_cloud_centers", 10, cloudCB);//接收点云  
  ros::spin();  
  return 0;  
} 
