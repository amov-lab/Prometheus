#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
//which contains the required definitions to load and store point clouds to PCD and other file formats.

main(int argc, char **argv)
{
    ros::init(argc, argv, "pc2_publisher");
    ros::NodeHandle nh("~");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/prometheus/pcl_groundtruth", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;


    std::string pcd_path;
    if (nh.getParam("pcd_path", pcd_path)) {
        ROS_INFO("Get the pcd_path : %s", pcd_path.c_str());
    } else {
        ROS_WARN("didn't find parameter pcd_path, use the default path");
        //std::string ros_path = ros::package::getPath("prometheus_gazebo");
        pcd_path = "/home/fly-vision/Prometheus/Simulator/gazebo_simulator/maps/obstacle.pcd";
    }

    pcl::io::loadPCDFile(pcd_path, cloud);
    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "world"; //this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
    ros::Rate loop_rate(2.0);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}