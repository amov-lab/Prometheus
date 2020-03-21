#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
//which contains the required definitions to load and store point clouds to PCD and other file formats.

main(int argc, char **argv)
{
    ros::init(argc, argv, "pc2_publisher");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("prometheus/pcl_groundtruth", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;
    pcl::io::loadPCDFile("/home/colin/Desktop/obstacle.pcd", cloud);
    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "world"; //this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}