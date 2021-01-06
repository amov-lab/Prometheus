/***************************************************************************************************************************
 * web_cam.cpp
 * Author: Jario
 * Update Time: 2020.1.12
 *
 * 说明: 读取usb摄像头，并将话题发布出来
 *      1. 【发布】图像话题
 *         /prometheus/camera/rgb/image_raw
***************************************************************************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "message_utils.h"


using namespace std;
using namespace cv;

// 使用cout打印消息
bool local_print = true;
// 使用prometheus_msgs::Message打印消息
bool message_print = false;
//【发布】调试消息
ros::Publisher message_pub;
std::string msg_node_name;

image_transport::Publisher image_pub;
//设置图像大小
// cv::Size image_size = Size(1920.0, 1080.0);


std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "web_cam");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    ros::Rate loop_rate(60);

    // 发布调试消息
    msg_node_name = "/prometheus/message/web_cam";
    message_pub = nh.advertise<prometheus_msgs::Message>(msg_node_name, 10);

    int camera_id = 0;
    if (nh.getParam("cam_id", camera_id)) {
        char camera_id_msg[256];
        sprintf(camera_id_msg, "camera id is %d", camera_id);
        if (local_print)
            cout << camera_id_msg << endl;
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, 
            camera_id_msg);
    }
    int camera_height(480), camera_width(640);
    int cam_h, cam_w;
    if (nh.getParam("cam_h", cam_h)) {
        if (cam_h > 0) camera_height = cam_h;
        char camera_id_msg[256];
        sprintf(camera_id_msg, "set capture height %d", camera_height);
        if (local_print)
            cout << camera_id_msg << endl;
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, 
            camera_id_msg);
    }
    if (nh.getParam("cam_w", cam_w)) {
        if (cam_w > 0) camera_width = cam_w;
        char camera_id_msg[256];
        sprintf(camera_id_msg, "set capture width %d", camera_width);
        if (local_print)
            cout << camera_id_msg << endl;
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, 
            camera_id_msg);
    }
    int framerate = 30;
    int rate;
    if (nh.getParam("framerate", rate)) {
        if (rate > 0) framerate = rate;
        char framerate_msg[256];
        sprintf(framerate_msg, "set framerate %d", framerate);
        if (local_print)
            cout << framerate_msg << endl;
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, 
            framerate_msg);
    }
    int flip_method = 0;
    int flip;
    if (nh.getParam("flip_method", flip)) {
        if (flip > 0) flip_method = flip;
        char flip_method_msg[256];
        sprintf(flip_method_msg, "set flip_method %d", flip_method);
        if (local_print)
            cout << flip_method_msg << endl;
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, 
            flip_method_msg);
    }

    int resize_h(0), resize_w(0);
    nh.getParam("resize_h", resize_h);
    nh.getParam("resize_w", resize_w);


    std::string pipeline = gstreamer_pipeline(
        cam_w,
        cam_h,
        resize_w,
        resize_h,
        framerate,
        flip_method
    );


    // 在这里修改发布话题名称
    image_pub = it.advertise("/prometheus/camera/rgb/image_raw", 1);

    // 用系统默认驱动读取摄像头0，使用其他摄像头ID，请在这里修改
    
    cv::VideoCapture cap;

    if (camera_id == 99) {
        cap.open(pipeline, cv::CAP_GSTREAMER);
    } else {
        cap.open(camera_id);
        // 设置摄像头分辨率
        cap.set(CAP_PROP_FRAME_WIDTH, camera_width);
        cap.set(CAP_PROP_FRAME_HEIGHT, camera_height);
    }

    cv::Mat frame;
    // 设置全屏
    // namedWindow("web_cam frame", CV_WINDOW_NORMAL);
    // setWindowProperty("web_cam frame", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    sensor_msgs::ImagePtr msg;

    long frame_cnt(0);
    bool cant_open(false);
    while (ros::ok())
    {
        cap >> frame;
        if (resize_w > 0 && resize_h > 0) {
            cv::resize(frame, frame, cv::Size(resize_w, resize_h));
        }
        if (!frame.empty())
        {
            // 改变图像大小并显示图片
            // resize(frame, frame, image_size);
            // imshow("web_cam frame", frame);
            // waitKey(5);
            if (0 == frame_cnt)
            {
                int fps = cap.get(CAP_PROP_FPS);
                if (local_print)
                    ROS_INFO("Camera %d opened, resolution: %d x %d, fps: %d", camera_id, frame.cols, frame.rows, fps);
            }
            // 设置图像帧格式->bgr8
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            // 将图像通过话题发布出去
            image_pub.publish(msg);
            frame_cnt += 1;
        }
        else
        {
            if (!cant_open)
            {
                cant_open = true;
                if (local_print)
                    ROS_WARN("Can not open camera %d.", camera_id);
                if (message_print)
                    pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, "Can not open camera");
            }
        }
        ros::spinOnce();
        // 按照设定的帧率延时，ros::Rate loop_rate(30)
        loop_rate.sleep();
    }

    cap.release();
}
