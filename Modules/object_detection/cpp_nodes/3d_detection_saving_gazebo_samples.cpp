/***************************************************************************************************************************
 * 3d_detection_saving_gazebo_samples.cpp
 * Author: Jario
 * Update Time: 2020.11.14
 *
 * 说明: 保存双目话题中的左图与右图，用于3d目标检测
 *      1. 【订阅】图像话题 (默认来自gazebo双目视觉话题)
 *         /P300_basic/stereo_camera/left/image_raw
 *         /P300_basic/stereo_camera/right/image_raw
 *      2. 【保存】按's'键保存双目图像
***************************************************************************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "message_utils.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;


// 回调函数中左图与右图变量
cv::Mat img_left, img_right;
// 左图和右图的存储路径
std::string img_left_path, img_right_path;

// 使用cout打印消息
bool local_print = true;
// 使用prometheus_msgs::Message打印消息
bool message_print = true;
//【发布】调试消息
ros::Publisher message_pub;
std::string msg_node_name;


void imageCallbackLeft(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        double image_time = msg->header.stamp.toSec();
        img_left = cv_bridge::toCvShare(msg,"bgr8")->image;
        cv::imshow("left", img_left);
        std::string image_name = std::to_string(image_time) + ".jpg";
        char key = cv::waitKey(1);
        if(key == 's')
        {
            cv::imwrite(img_left_path + "/" + image_name, img_left);
            cv::imwrite(img_right_path + "/" + image_name, img_right);
        }
    }
    catch(cv_bridge::Exception &e)
    {

    }
}


void imageCallbackRight(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        img_right =  cv_bridge::toCvShare(msg,"bgr8")->image;
        // cv::imshow("right", img_right);
        // cv::waitKey(1);
    }
    catch(cv_bridge::Exception &e)
    {

    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_detection_saving_gazebo_samples");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // 发布调试消息
    msg_node_name = "/prometheus/message/stereo_detection_saving_gazebo_samples";
    message_pub = nh.advertise<prometheus_msgs::Message>(msg_node_name, 10);

    std::string camera_left_topic, camera_right_topic;
    if (nh.getParam("camera_left_topic", camera_left_topic)) {
        if (local_print)
            ROS_INFO("camera_left_topic is %s", camera_left_topic.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "camera_left_topic is" + camera_left_topic);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter camera_left_topic");
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, "didn't find parameter camera_left_topic");
        camera_left_topic = "/P300_basic/stereo_camera/left/image_raw";
    }
    if (nh.getParam("camera_right_topic", camera_right_topic)) {
        if (local_print)
            ROS_INFO("camera_right_topic is %s", camera_right_topic.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "camera_right_topic is" + camera_right_topic);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter camera_right_topic");
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, "didn't find parameter camera_right_topic");
        camera_right_topic = "/P300_basic/stereo_camera/right/image_raw";
    }


    if (nh.getParam("img_left_path", img_left_path)) {
        if (local_print)
            ROS_INFO("img_left_path is %s", img_left_path.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "img_left_path is" + img_left_path);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter img_left_path");
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, "didn't find parameter img_left_path");
        img_left_path = "/tmp/save_images/left/";
    }
    if (nh.getParam("img_right_path", img_right_path)) {
        if (local_print)
            ROS_INFO("img_right_path is %s", img_right_path.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "img_right_path is" + img_right_path);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter img_right_path");
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, "didn't find parameter img_right_path");
        img_right_path = "/tmp/save_images/right/";
    }

    image_transport::Subscriber sub_left = it.subscribe(camera_left_topic, 1,imageCallbackLeft);
    image_transport::Subscriber sub_right = it.subscribe(camera_right_topic, 1,imageCallbackRight);

    ros::spin();     
}
