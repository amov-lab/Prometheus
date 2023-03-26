#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>

using namespace std;

ros::Publisher scan_pub, scan_point_cloud_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan)
{
  // 参考网页:http://wiki.ros.org/laser_geometry
  // sensor_msgs::LaserScan 转为 sensor_msgs::PointCloud2 格式
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud2 input_laser_scan;
  projector_.projectLaser(*laser_scan, input_laser_scan);
  
  input_laser_scan.header.stamp = ros::Time::now(); //时间戳
  scan_point_cloud_pub.publish(input_laser_scan);

  sensor_msgs::LaserScan scan_with_system_time =  *laser_scan;
  scan_with_system_time.header.stamp = ros::Time::now(); //时间戳
  scan_pub.publish(scan_with_system_time);
}

//主函数
int main(int argc, char** argv)
{
    //ROS初始化,设定节点名
    ros::init(argc , argv, "laser_to_pointcloud");
    //创建句柄
    ros::NodeHandle n;

    int uav_id;
    //获取起飞高度参数
    ros::param::get("~uav_id", uav_id);
    uav_id = 1;
    
    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/uav" + std::to_string(uav_id) + "/scan", 10, scanCallback);

    scan_pub = n.advertise<sensor_msgs::LaserScan>("/uav" + std::to_string(uav_id) + "/prometheus/scan_in_system_time", 10);
    scan_point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/prometheus/scan_point_cloud", 10);

    ros::Rate r(100.0);

    while(ros::ok())
    {
        //调用一次回调函数
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}