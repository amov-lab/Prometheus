#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// 发布点云
ros::Publisher pointcloud_pub;

// 回调函数，将PointCloud2转换为PointCloud，修改y、z后再发布
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Step 1: Convert PointCloud2 to PointCloud
    sensor_msgs::PointCloud point_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*input, point_cloud);

    // Step 2: Iterate through the points and modify y and z
    for (size_t i = 0; i < point_cloud.points.size(); ++i) {
        point_cloud.points[i].x = point_cloud.points[i].x;
        point_cloud.points[i].y = point_cloud.points[i].y;
        point_cloud.points[i].z = -point_cloud.points[i].z;
    }

    // Step 3: Convert PointCloud back to PointCloud2
    sensor_msgs::PointCloud2 output;
    sensor_msgs::convertPointCloudToPointCloud2(point_cloud, output);
    output.header.frame_id = "uav1/d435i_link";
    // Step 4: Publish the modified PointCloud2
    pointcloud_pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_converter");
    ros::NodeHandle nh;

    // 订阅PointCloud2话题
    ros::Subscriber pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/airsim_node/uva1/lidarPointCloud2/LidarSensor1", 10, pointCloudCallback);

    // 发布修改后的PointCloud2话题
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/uav1/D435i/points", 10);

    ros::spin();
    return 0;
}

