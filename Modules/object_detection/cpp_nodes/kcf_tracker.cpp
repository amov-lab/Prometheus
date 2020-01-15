/***************************************************************************************************************************
 * kcf_tracker.cpp
 * Author: Jario
 * Update Time: 2020.1.14
 *
 * 说明: KCF目标跟踪程序
 *      1. 【订阅】图像话题 (默认来自web_cam)
 *         /prometheus/camera/rgb/image_raw
 *      2. 【发布】目标位置，发布话题见 Prometheus/Modules/msgs/msg/DetectionInfo.msg
 *         /prometheus/target
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
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml.hpp>

#include "kcftracker.hpp"

using namespace std;
using namespace cv;


#define MARKER_SIZE 0.18
#define F1 300
#define F2 300
#define C1 320
#define C2 240

static const std::string RGB_WINDOW = "RGB Image window";

// 相机话题中的图像同步相关变量
int frame_width, frame_height;
std_msgs::Header image_header;
cv::Mat cam_image_copy;
boost::shared_mutex mutex_image_callback;
bool image_status = false;
boost::shared_mutex mutex_image_status;


// 图像接收回调函数，接收web_cam的话题，并将图像保存在cam_image_copy中
void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("[KCFTracker] USB image received.");

    cv_bridge::CvImagePtr cam_image;

    try {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_header = msg->header;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
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
image_transport::Subscriber imageSubscriber_;
ros::Publisher pose_pub;


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


bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);


int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracker_ros");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); 
    ros::Rate loop_rate(30);
    
    // 接收图像的话题
    imageSubscriber_ = it.subscribe("/camera/rgb/image_raw", 1, cameraCallback);

    // 跟踪结果，xyz
    pose_pub = nh.advertise<prometheus_msgs::DetectionInfo>("/vision/target", 1);
    
    sensor_msgs::ImagePtr msg_ellipse;

    std::string ros_path = ros::package::getPath("prometheus_detection");
    cout << "DETECTION_PATH: " << ros_path << endl;
    //读取参数文档camera_param.yaml中的参数值；
    YAML::Node camera_config = YAML::LoadFile(ros_path + "/config/camera_param.yaml");
    //相机内部参数
    double fx = camera_config["fx"].as<double>();
    double fy = camera_config["fy"].as<double>();
    double cx = camera_config["x0"].as<double>();
    double cy = camera_config["y0"].as<double>();
    //相机畸变系数
    double k1 = camera_config["k1"].as<double>();
    double k2 = camera_config["k2"].as<double>();
    double p1 = camera_config["p1"].as<double>();
    double p2 = camera_config["p2"].as<double>();
    double k3 = camera_config["k3"].as<double>();

    const auto wait_duration = std::chrono::milliseconds(2000);

    cv::namedWindow(RGB_WINDOW);
    cv::setMouseCallback(RGB_WINDOW, onMouse, 0);
    while (ros::ok())
    {
        while (!getImageStatus()) 
        {
            printf("Waiting for image.\n");
            std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
        }

        Mat frame;
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            frame = cam_image_copy.clone();
        }
        static bool need_tracking_det = false;

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
        }

        imshow(RGB_WINDOW, frame);
        waitKey(5);
        

        ros::spinOnce();
        loop_rate.sleep();
    }
}




