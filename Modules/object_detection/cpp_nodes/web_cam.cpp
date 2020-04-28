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
#include "prometheus_control_utils.h"


using namespace prometheus_control_utils;
using namespace std;
using namespace cv;

// 使用cout打印消息
bool local_print = false;
// 使用prometheus_msgs::Message打印消息
bool message_print = true;
//【发布】调试消息
ros::Publisher message_pub;
std::string msg_node_name;

image_transport::Publisher image_pub;
//设置图像大小
// cv::Size image_size = Size(1920.0, 1080.0);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "web_cam");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Rate loop_rate(60);

    // 发布调试消息
    msg_node_name = "/prometheus/message/web_cam";
    message_pub = nh.advertise<prometheus_msgs::Message>(msg_node_name, 10);

    // 在这里修改发布话题名称
    image_pub = it.advertise("/prometheus/camera/rgb/image_raw", 1);

    // 用系统默认驱动读取摄像头0，使用其他摄像头ID，请在这里修改
    int camera_id = 0;
    cv::VideoCapture cap(camera_id);
    // 设置摄像头分辨率
    // cap.set(CAP_PROP_FRAME_WIDTH, image_size.height);
    // cap.set(CAP_PROP_FRAME_HEIGHT, image_size.width);
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
