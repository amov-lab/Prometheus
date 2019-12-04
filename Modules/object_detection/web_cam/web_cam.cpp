#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>  
#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


image_transport::Publisher image_pub;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "web_cam");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); 
    ros::Rate loop_rate(30);
    // 在这里修改发布话题名称
    image_pub = it.advertise("/camera/rgb/image_raw", 1);

    // 用系统默认驱动读取摄像头0，使用其他摄像头ID，请在这里修改
    cv::VideoCapture cap(0);
    cv::Mat frame;

    sensor_msgs::ImagePtr msg;

    while (ros::ok())
	{
        cap >> frame;
        if (!frame.empty())
        {
            // 设置图像帧格式->bgr8
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            // 将图像通过话题发布出去
            image_pub.publish(msg); 
        }
        ros::spinOnce();
        // 按照设定的帧率延时，ros::Rate loop_rate(30)
        loop_rate.sleep();
    }

    cap.release();
}




