/***************************************************************************************************************************
 * aruco_det.cpp
 * Author: Jario
 * Update Time: 2020.1.14
 *
 * 说明: 单个二维码识别程序，可识别的二维码在Prometheus/Modules/object_detection/config/aruco_images文件夹中
 *      视野里只允许存在一个二维码 且二维码的字典类型要对应
 *      默认二维码的边长为0.2m
 *      1. 【订阅】图像话题 (默认来自web_cam)
 *         /prometheus/camera/rgb/image_raw
 *      2. 【发布】目标位置，发布话题见 Prometheus/Modules/msgs/msg/DetectionInfo.msg
 *         /prometheus/object_detection/aruco_det
 *      3. 【发布】检测结果的可视化图像话题
 *         /prometheus/camera/rgb/image_aruco_det
***************************************************************************************************************************/
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
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/calib3d.hpp"
#include "message_utils.h"


using namespace std;
using namespace cv;

//【订阅】输入图像
image_transport::Subscriber image_subscriber;
//【发布】检测得到的位置与姿态信息
ros::Publisher pose_pub;
//【发布】输入检测结果图像
image_transport::Publisher aruco_pub;

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


void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
    double x1 = x;//将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
    double y1 = y;
    double rz = thetaz * CV_PI / 180;
    outx = cos(rz) * x1 - sin(rz) * y1;
    outy = sin(rz) * x1 + cos(rz) * y1;
}
void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz)
{
    double x1 = x;
    double z1 = z;
    double ry = thetay * CV_PI / 180;
    outx = cos(ry) * x1 + sin(ry) * z1;
    outz = cos(ry) * z1 - sin(ry) * x1;
}
void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
    double y1 = y;//将变量拷贝一次，保证&y == &y这种情况下也能计算正确
    double z1 = z;
    double rx = thetax * CV_PI / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}
static float static_depth = 0;
static float static_real_x = 0;
static float static_real_y = 0;


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
    if (local_print)
        ROS_DEBUG("[ArucoDetector] USB image received.");

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

void switchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    is_suspanded = !(bool)msg->data;
    // cout << is_suspanded << endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_det");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    // 更新频率为30HZ
    ros::Rate loop_rate(30);
    //【发布】识别
    pose_pub = nh.advertise<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/aruco_det", 1);


    // 发布调试消息
    msg_node_name = "/prometheus/message/aruco_det";
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

    // 接收图像的话题
    image_subscriber = it.subscribe(camera_topic.c_str(), 1, cameraCallback);
    // 发布ArUco检测结果的话题
    aruco_pub = it.advertise("/prometheus/camera/rgb/image_aruco_det", 1);

    bool switch_state = is_suspanded;
    // 接收开关话题
    switch_subscriber = nh.subscribe("/prometheus/switch/aruco_det", 10, switchCallback);

    sensor_msgs::ImagePtr msg_ellipse;
    // 开启编号为0的摄像头
    // cv::VideoCapture cap(0);
	cv::Mat frame;

    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;

    std::string ros_path = ros::package::getPath("prometheus_detection");
    if (local_print)
        cout << "DETECTION_PATH: " << ros_path << endl;
    if (message_print)
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "DETECTION_PATH: " + ros_path);
    //读取参数文档camera_param.yaml中的参数值；
    YAML::Node camera_config = YAML::LoadFile(ros_path + "/config/" + camera_info);
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

    double aruco_det_len = camera_config["aruco_det_len"].as<double>();

    //相机内参
    camera_matrix = cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0]=fx;
    camera_matrix.ptr<double>(0)[2]=cx;
    camera_matrix.ptr<double>(1)[1]=fy;
    camera_matrix.ptr<double>(1)[2]=cy;
    camera_matrix.ptr<double>(2)[2]=1.0f;
    
    //相机畸变参数
    distortion_coefficients=cv::Mat(5,1,CV_64FC1,cv::Scalar::all(0));
    distortion_coefficients.ptr<double>(0)[0]=k1;
    distortion_coefficients.ptr<double>(1)[0]=k2;
    distortion_coefficients.ptr<double>(2)[0]=p1;
    distortion_coefficients.ptr<double>(3)[0]=p2;
    distortion_coefficients.ptr<double>(4)[0]=k3;

    //ArUco Marker字典选择
    Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    vector<double> rv(3),tv(3);
    cv::Mat rvec(rv),tvec(tv);
    const auto wait_duration = std::chrono::milliseconds(2000);

    // 生成Aruco码
    // 保存在Prometheus/Modules/object_detection/config/aruco_images文件夹中
    /*******************************************************************************************
    cv::Mat markerImage;
    cv::Mat markerImagePad = cv::Mat::ones(500, 500,  CV_8UC1) * 255;
    for (int i=0; i<250; i++)
    {
        cv::aruco::drawMarker(dictionary, i, 400, markerImage, 1);
        markerImage.copyTo(markerImagePad.rowRange(50, 450).colRange(50, 450));
        cv::imshow("markerImagePad", markerImagePad);
        cv::waitKey(100);
        char img_fn[256];
        sprintf(img_fn, "Prometheus/Modules/object_detection/config/aruco_images/DICT_6X6_250_%d.png", i);
        cv::imwrite(img_fn, markerImagePad);
    }
    *******************************************************************************************/


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

        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            frame = cam_image_copy.clone();
        }

        if (!frame.empty())
        {
            std::vector<int> markerids;
            vector<vector<Point2f> > markerCorners,rejectedCandidate;
            Ptr<cv::aruco::DetectorParameters> parameters=cv::aruco::DetectorParameters::create();
            cv::aruco::detectMarkers(frame,dictionary,markerCorners,markerids,parameters,rejectedCandidate);

            bool deted = false;
            if (markerids.size()>0)
            {
                cv::Mat RoteM, TransM;
                cv::Point3f Theta_C2W;
                cv::Point3f Theta_W2C;
                cv::Point3f Position_OcInW;

                cv::aruco::estimatePoseSingleMarkers(markerCorners,aruco_det_len,camera_matrix,distortion_coefficients,rvec,tvec);

                prometheus_msgs::DetectionInfo pose_now;
                pose_now.header.stamp = ros::Time::now();
                pose_now.detected = true;
                pose_now.frame = 0;
                pose_now.position[0] = tvec.ptr<double>(0)[0];
                pose_now.position[1] = tvec.ptr<double>(0)[1];
                pose_now.position[2] = tvec.ptr<double>(0)[2];

                static_real_x = tvec.ptr<double>(0)[0];
                static_real_y = tvec.ptr<double>(0)[1];
                static_depth  = tvec.ptr<double>(0)[2];

                if (local_print) {
                    cout << "flag_detected: " << int(pose_now.detected) <<endl;
                    cout << "pos_target: [X Y Z] : " << " " << pose_now.position[0] << " [m] "<< pose_now.position[1] <<" [m] "<< pose_now.position[2] <<" [m] "<<endl;
                }
                pose_pub.publish(pose_now);
                deted=true;
            }
            if (!deted)
            {
                prometheus_msgs::DetectionInfo pose_now;
                pose_now.header.stamp = ros::Time::now();
                pose_now.detected = false;
                pose_now.frame = 0;
                pose_now.position[0] = static_real_x;
                pose_now.position[1] = static_real_y;
                pose_now.position[2] = static_depth;

                pose_pub.publish(pose_now);

                if (local_print) {
                    cout << "flag_detected: " << int(pose_now.detected) <<endl;
                    cout << "pos_target: [X Y Z] : " << " " << pose_now.position[0] << " [m] "<< pose_now.position[1] <<" [m] "<< pose_now.position[2] <<" [m] "<<endl;
                }
            }

            Mat img_copy;
            frame.copyTo(img_copy);
            cv::aruco::drawDetectedMarkers(img_copy,markerCorners,markerids);
            msg_ellipse = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_copy).toImageMsg();
            aruco_pub.publish(msg_ellipse);
            cv::waitKey(1);

		}
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

	  // cap.release();

}




