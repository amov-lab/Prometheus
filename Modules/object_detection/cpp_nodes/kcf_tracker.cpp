/***************************************************************************************************************************
 * kcf_tracker.cpp
 * Author: Jario
 * Update Time: 2020.1.14
 *
 * 说明: KCF目标跟踪程序
 *      1. 【订阅】图像话题 (默认来自web_cam)
 *         /prometheus/camera/rgb/image_raw
 *      2. 【发布】目标位置，发布话题见 Prometheus/Modules/msgs/msg/DetectionInfo.msg
 *         /prometheus/object_detection/kcf_tracker
***************************************************************************************************************************/
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <image_transport/image_transport.h>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml.hpp>

#include "kcftracker.hpp"
#include "message_utils.h"


using namespace std;
using namespace cv;


static const std::string RGB_WINDOW = "RGB Image window";

// 相机话题中的图像同步相关变量
int frame_width, frame_height;
std_msgs::Header image_header;
cv::Mat cam_image_copy;
boost::shared_mutex mutex_image_callback;
bool image_status = false;
boost::shared_mutex mutex_image_status;

//【订阅】输入开关量
ros::Subscriber switch_subscriber;
// 接收消息，允许暂停检测
bool is_suspanded = false;
// 使用cout打印消息
bool local_print = false;
// 使用prometheus_msgs::Message打印消息
bool message_print = true;
//【发布】调试消息
ros::Publisher message_pub;
std::string msg_node_name;


// 图像接收回调函数，接收web_cam的话题，并将图像保存在cam_image_copy中
void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (local_print)
        ROS_DEBUG("[KCFTracker] USB image received.");

    cv_bridge::CvImagePtr cam_image;

    try {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_header = msg->header;
    } catch (cv_bridge::Exception& e) {
        if (local_print)
            ROS_ERROR("cv_bridge exception: %s", e.what());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::ERROR, msg_node_name, "cv_bridge exception");
        return;
    }

    if (cam_image) {
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            cam_image_copy = cam_image->image.clone();
        }
        {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutex_image_status);
            image_status = true;
        }
        frame_width = cam_image->image.size().width;
        frame_height = cam_image->image.size().height;
    }
    return;
}

// 用此函数查看是否收到图像话题
bool getImageStatus(void)
{
    boost::shared_lock<boost::shared_mutex> lock(mutex_image_status);
    return image_status;
}

//! ROS subscriber and publisher.
//【订阅】输入图像
image_transport::Subscriber imageSubscriber_;
//【发布】目标位置
ros::Publisher pose_pub;
prometheus_msgs::DetectionInfo pose_now;

cv::Rect selectRect;
cv::Point origin;
cv::Rect result;

bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;

void onMouse(int event, int x, int y, int, void*)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);        
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);   
        selectRect.height = abs(y - origin.y);
        selectRect &= cv::Rect(0, 0, frame_width, frame_height);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        bBeginKCF = false;  
        select_flag = true; 
        origin = cv::Point(x, y);       
        selectRect = cv::Rect(x, y, 0, 0);  
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        if (selectRect.width*selectRect.height < 64)
        {
            ;
        }
        else
        {
            select_flag = false;
            bRenewROI = true;    
        } 
    }
}


void switchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    is_suspanded = !(bool)msg->data;
    // cout << is_suspanded << endl;
}


bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kcf_tracker");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh); 
    ros::Rate loop_rate(30);

    // 发布调试消息
    msg_node_name = "/prometheus/message/kcf_tracker";
    message_pub = nh.advertise<prometheus_msgs::Message>(msg_node_name, 10);

    std::string camera_topic, camera_info;
    if (nh.getParam("camera_topic", camera_topic)) {
        if (local_print)
            ROS_INFO("camera_topic is %s", camera_topic.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "camera_topic is" + camera_topic);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter camera_topic");
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, "didn't find parameter camera_topic");
        camera_topic = "/prometheus/camera/rgb/image_raw";
    }

    if (nh.getParam("camera_info", camera_info)) {
        if (local_print)
            ROS_INFO("camera_info is %s", camera_info.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "camera_info is" + camera_info);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter camera_info");
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, "didn't find parameter camera_info");
        camera_info = "camera_param.yaml";
    }

    bool switch_state = is_suspanded;
    // 接收开关话题
    switch_subscriber = nh.subscribe("/prometheus/switch/kcf_tracker", 10, switchCallback);
    
    // 接收图像的话题
    imageSubscriber_ = it.subscribe(camera_topic.c_str(), 1, cameraCallback);

    // 跟踪结果，xyz
    pose_pub = nh.advertise<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/kcf_tracker", 1);
    
    sensor_msgs::ImagePtr msg_ellipse;

    std::string ros_path = ros::package::getPath("prometheus_detection");
    if (local_print)
        cout << "DETECTION_PATH: " << ros_path << endl;
    if (message_print)
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "DETECTION_PATH: " + ros_path);
    // 读取参数文档camera_param.yaml中的参数值；
    YAML::Node camera_config = YAML::LoadFile(camera_info);
    // 相机内部参数
    double fx = camera_config["fx"].as<double>();
    double fy = camera_config["fy"].as<double>();
    double cx = camera_config["x0"].as<double>();
    double cy = camera_config["y0"].as<double>();
    // 相机畸变系数
    double k1 = camera_config["k1"].as<double>();
    double k2 = camera_config["k2"].as<double>();
    double p1 = camera_config["p1"].as<double>();
    double p2 = camera_config["p2"].as<double>();
    double k3 = camera_config["k3"].as<double>();

    double kcf_tracker_h = camera_config["kcf_tracker_h"].as<double>();

    const auto wait_duration = std::chrono::milliseconds(2000);

    cv::namedWindow(RGB_WINDOW);
    cv::setMouseCallback(RGB_WINDOW, onMouse, 0);
    float last_x(0), last_y(0), last_z(0), last_ax(0), last_ay(0);

    while (ros::ok())
    {
        while (!getImageStatus() && ros::ok()) 
        {
            if (local_print)
                cout << "Waiting for image." << endl;
            if (message_print)
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "Waiting for image.");
            std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
        }

        if (switch_state != is_suspanded)
        {
            switch_state = is_suspanded;
            if (!is_suspanded)
            {
                if (local_print)
                    cout << "Start Detection." << endl;
                if (message_print)
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "Start Detection.");
            }
            else
            {
                if (local_print)
                    cout << "Stop Detection." << endl;
                if (message_print)
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "Stop Detection.");
            }
        }

        if (!is_suspanded)
        {
        Mat frame;
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            frame = cam_image_copy.clone();
        }
        static bool need_tracking_det = false;

        bool detected = false;
        if (bRenewROI)
        {
            tracker.init(selectRect, frame);
            cv::rectangle(frame, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);
            bRenewROI = false;
            bBeginKCF = true;
        }
        else if (bBeginKCF)
        {
            result = tracker.update(frame);
            cv::rectangle(frame, result, cv::Scalar(255, 0, 0), 2, 8, 0);

            // 将解算后的位置发给控制端
            detected = true;
            pose_now.header.stamp = ros::Time::now();
            pose_now.detected = true;
            pose_now.frame = 0;
            double depth = kcf_tracker_h / result.height * fy;
            double cx = result.x + result.width / 2 - frame.cols / 2;
            double cy = result.y + result.height / 2 - frame.rows / 2;
            pose_now.position[0] = depth * cx / fx;
            pose_now.position[1] = depth * cy / fy;
            pose_now.position[2] = depth;

            pose_now.sight_angle[0] = cx / (frame.cols / 2) * atan((frame.cols / 2) / fx);
            pose_now.sight_angle[1] = cy / (frame.rows / 2) * atan((frame.rows / 2) / fy);

            last_x = pose_now.position[0];
            last_y = pose_now.position[1];
            last_z = pose_now.position[2];
            last_ax = pose_now.sight_angle[0];
            last_ay = pose_now.sight_angle[1];
        }

        if (!detected)
        {
            pose_now.header.stamp = ros::Time::now();
            pose_now.detected = false;
            pose_now.frame = 0;
            pose_now.position[0] = last_x;
            pose_now.position[1] = last_y;
            pose_now.position[2] = last_z;
            pose_now.sight_angle[0] = last_ax;
            pose_now.sight_angle[1] = last_ay;
        }

        pose_pub.publish(pose_now);

        imshow(RGB_WINDOW, frame);
        }

        waitKey(5);
        ros::spinOnce();
        loop_rate.sleep();
    }
}




