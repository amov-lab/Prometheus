#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

ros::Publisher map_pub;
ros::Publisher odom_pub;
ros::Publisher waypoint_pub;

bool is_load_map{false};
sensor_msgs::PointCloud2 cloud_map_msg;
nav_msgs::Path waypoints;
nav_msgs::Odometry init_odom;

void visualizer_one_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string name);
void load_map(void){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
    
    ros::Time begin = ros::Time::now();		//计算读取时间，耗时比较长，但是发布耗时小于1s，发布频率可以为1
    ROS_INFO("begin:%f",begin.toSec());

    // pcl::io::loadPCDFile("/home/taojiang/Desktop/cloud2.pcd",*cloud_map);
    pcl::io::loadPCDFile("/home/taojiang/Desktop/cloud0115.pcd",*cloud_map);
    visualizer_one_points(cloud_map,"map");

    ros::Time load = ros::Time::now();
    ROS_INFO("load-begin=%f -%f=%f",load.toSec(),begin.toSec(),load.toSec()-begin.toSec());

    pcl::toROSMsg(*cloud_map,cloud_map_msg);		//转成ros消息
    cloud_map_msg.header.frame_id = "world";		//rviz显示需要fixed_frame
    is_load_map = true;
}

void pcdpubCallback(const ros::TimerEvent& e) {
    if (!is_load_map) {
        printf("no point map!\n");
        return;
    }
    map_pub.publish(cloud_map_msg);

}

void omdpubCallback(const ros::TimerEvent& e) {

    //we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();;
    odom.header.frame_id = "world";

    //set the position
    odom.pose.pose.position.x = init_odom.pose.pose.position.x;
    odom.pose.pose.position.y = init_odom.pose.pose.position.y;
    odom.pose.pose.position.z = init_odom.pose.pose.position.z;

    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = 0;
    odom_quat.y = 0;
    odom_quat.z = 0;
    odom_quat.w = 0;
    odom.pose.pose.orientation = odom_quat;

    //publish the message
    odom_pub.publish(odom);

}

void waypointpubCallback(const ros::TimerEvent &e){
    geometry_msgs::PoseStamped pt;
    pt.header.frame_id = "world";
    pt.header.stamp = ros::Time::now();
    pt.pose.position.x = 0.0;
    pt.pose.position.y = -2.0;
    pt.pose.position.z = 1.0;

    // waypoints.poses.clear();
    // waypoints.poses.push_back(pt);
    // waypoints.header.frame_id = std::string("world");
    // waypoints.header.stamp = ros::Time::now();
    // waypoint_pub.publish(waypoints);
    waypoint_pub.publish(pt);
    ROS_INFO("--- test_planning_static: pub wayoint successful!---");
}

//pcl显示点云
void visualizer_one_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string name)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(name));

    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud,0,0,255);
    viewer->addPointCloud(cloud,cloud_color,name);

    viewer->spinOnce(100);			//while(!viewer->wasStopped())会卡在这
}

void waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg);

int main(int argc,char** argv)
{
    ros::init(argc,argv,"pub_map_odom");
    ros::NodeHandle node_("~");


    odom_pub = node_.advertise<nav_msgs::Odometry>("/planning/odom_world", 50);
    map_pub =node_.advertise<sensor_msgs::PointCloud2>("/planning/global_point_cloud",1);
    // waypoint_pub = node_.advertise<geometry_msgs::PoseStamped>("/planning/goal", 50);
    waypoint_pub = node_.advertise<geometry_msgs::PoseStamped>("/planning/waypoint", 50);

    ros::Subscriber waypoint_sub_ = node_.subscribe("/planning/goal", 1, &waypointCallback);

    // pub1 = n.advertise<nav_msgs::Path>("waypoints", 50);
    ros::Timer pub_odom_timer_ = node_.createTimer(ros::Duration(0.05), &omdpubCallback);
    ros::Timer pub_pcd_timer = node_.createTimer(ros::Duration(5.0), &pcdpubCallback);
    ros::Timer pub_waypoint_timer = node_.createTimer(ros::Duration(2.0), &waypointpubCallback);

    node_.param("test/init_odom_x", init_odom.pose.pose.position.x, 0.0);
    node_.param("test/init_odom_y", init_odom.pose.pose.position.y, 0.0);
    node_.param("test/init_odom_z", init_odom.pose.pose.position.z, 1.0);

    
    load_map();
    ros::spin();

    return 0;
}

void waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  cout << "[waypointCallback]: receive goal!" << endl;
  ROS_INFO("waypoint: [%f, %f, %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}
