// c++
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

//! Camera related parameters.
int frameWidth_;
int frameHeight_;

std_msgs::Header imageHeader_;
cv::Mat camImageCopy_;
boost::shared_mutex mutexImageCallback_;
bool imageStatus_ = false;
boost::shared_mutex mutexImageStatus_;


// 图像接收回调函数，接收web_cam的话题，并将图像保存在camImageCopy_中
void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("[EllipseDetector] USB image received.");

    cv_bridge::CvImagePtr cam_image;

    try {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        imageHeader_ = msg->header;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cam_image) {
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            camImageCopy_ = cam_image->image.clone();
        }
        {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
            imageStatus_ = true;
        }
        frameWidth_ = cam_image->image.size().width;
        frameHeight_ = cam_image->image.size().height;
    }
    return;
}

// 用此函数查看是否收到图像话题
bool getImageStatus(void)
{
    boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
    return imageStatus_;
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
        selectRect &= cv::Rect(0, 0, frameWidth_, frameHeight_);
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
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            frame = camImageCopy_.clone();
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




