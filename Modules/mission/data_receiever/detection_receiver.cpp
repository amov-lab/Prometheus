#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>

#include "mission_utils.h"
#include "message_utils.h"

#include <fstream>

using namespace std;
using namespace Eigen;

Detection_result landpad_det1; // 检测结果
Detection_result landpad_det2; // 检测结果
Detection_result landpad_det3; // 检测结果

std::ofstream out1("/home/colin/uav1_detection.txt", std::ios::app);
std::ofstream out2("/home/colin/uav2_detection.txt", std::ios::app);
std::ofstream out3("/home/colin/uav3_detection.txt", std::ios::app);
void landpad_det_cb1(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{

    landpad_det1.object_name = "landpad_u1";
    landpad_det1.Detection_info = *msg;
    double x = landpad_det1.Detection_info.position[0];
    double y = landpad_det1.Detection_info.position[1];
    double z = landpad_det1.Detection_info.position[2];
    double ag_x = landpad_det1.Detection_info.position[0];
    double ag_y = landpad_det1.Detection_info.position[1];
    double ag_z = landpad_det1.Detection_info.position[2];
    out1 << x << "\t\t" << y << z << "\t\t";
    out1.close();
}
void landpad_det_cb2(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    landpad_det2.object_name = "landpad_u2";
    landpad_det2.Detection_info = *msg;
    double x = landpad_det2.Detection_info.position[0];
    double y = landpad_det2.Detection_info.position[1];
    double z = landpad_det2.Detection_info.position[2];
    double ag_x = landpad_det1.Detection_info.position[0];
    double ag_y = landpad_det1.Detection_info.position[1];
    double ag_z = landpad_det1.Detection_info.position[2];
}
void landpad_det_cb3(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    landpad_det3.object_name = "landpad_u3";
    landpad_det3.Detection_info = *msg;
    double x = landpad_det3.Detection_info.position[0];
    double y = landpad_det3.Detection_info.position[1];
    double z = landpad_det3.Detection_info.position[2];
    double ag_x = landpad_det1.Detection_info.position[0];
    double ag_y = landpad_det1.Detection_info.position[1];
    double ag_z = landpad_det1.Detection_info.position[2];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection_receiver");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    ros::Subscriber landpad_det_sub_u1 = nh.subscribe<prometheus_msgs::DetectionInfo>("/uav1/prometheus/object_detection/landpad_det", 10, landpad_det_cb1);
    ros::Subscriber landpad_det_sub_u2 = nh.subscribe<prometheus_msgs::DetectionInfo>("uav2/prometheus/object_detection/landpad_det", 10, landpad_det_cb2);
    ros::Subscriber landpad_det_sub_u3 = nh.subscribe<prometheus_msgs::DetectionInfo>("uav3/prometheus/object_detection/landpad_det", 10, landpad_det_cb3);
    ros::spin();

    return 0;
}