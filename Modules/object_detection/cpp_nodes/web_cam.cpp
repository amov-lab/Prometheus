#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

// 使用cout打印消息
bool local_print = true;

image_transport::Publisher image_pub;
//设置图像大小
// cv::Size image_size = Size(1920.0, 1080.0);

std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

std::string gstreamer_pipeline_veye327(int display_width, int display_height)
{
    return "nvv4l2camerasrc ! video/x-raw(memory:NVMM), format=(string)UYVY, width=(int)1920, height=(int)1080 ! \
            nvvidconv ! video/x-raw(memory:NVMM), format=(string)I420 ! \
            nvvidconv ! video/x-raw, format=(string)BGRx ! \
            videoconvert ! video/x-raw, format=(string)BGR ! \
            videoscale! video/x-raw,width=" +
           std::to_string(display_width) +
           ",height=" + std::to_string(display_width) + "! appsink";
}

std::string gstreamer_pipeline_G1(std::string web_cam_ip, int display_width,
                                  int display_height)
{
    return "rtspsrc location=rtsp://" + web_cam_ip +
           ":/H264?W=1920&H=1080&FPS=30&BR=4900000 latency=100 \
                    caps='application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264,width=1920,height=1080,framerate=30/1' !\
                    rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! \
                    video/x-raw, width=(int)" +
           std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! \
                    videoconvert ! appsink sync=false";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "web_cam");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    ros::Rate loop_rate(60);

    int camera_id = 0;
    if (nh.getParam("cam_id", camera_id))
    {
        char camera_id_msg[256];
        sprintf(camera_id_msg, "camera id is %d", camera_id);
        if (local_print)
            cout << camera_id_msg << endl;
    }
    int camera_type = 0; // 0: web_cam, 1: mipi_cam, 2: arducam 3:VEYE327 4: G1 gimbal
    if (nh.getParam("camera_type", camera_type))
    {
        char camera_type_msg[256];
        sprintf(camera_type_msg, "camera type is %d", camera_type);
        if (local_print)
            cout << camera_type_msg << endl;
    }
    int camera_height(480), camera_width(640);
    int cam_h, cam_w;
    if (nh.getParam("cam_h", cam_h))
    {
        if (cam_h > 0)
            camera_height = cam_h;
        char camera_id_msg[256];
        sprintf(camera_id_msg, "set capture height %d", camera_height);
        if (local_print)
            cout << camera_id_msg << endl;
    }
    if (nh.getParam("cam_w", cam_w))
    {
        if (cam_w > 0)
            camera_width = cam_w;
        char camera_id_msg[256];
        sprintf(camera_id_msg, "set capture width %d", camera_width);
        if (local_print)
            cout << camera_id_msg << endl;
    }
    int framerate = 30;
    int rate;
    if (nh.getParam("framerate", rate))
    {
        if (rate > 0)
            framerate = rate;
        char framerate_msg[256];
        sprintf(framerate_msg, "set framerate %d", framerate);
        if (local_print)
            cout << framerate_msg << endl;
    }
    int flip_method = 0;
    int flip;
    if (nh.getParam("flip_method", flip))
    {
        if (flip > 0)
            flip_method = flip;
        char flip_method_msg[256];
        sprintf(flip_method_msg, "set flip_method %d", flip_method);
        if (local_print)
            cout << flip_method_msg << endl;
    }

    int resize_h(0), resize_w(0);
    nh.getParam("resize_h", resize_h);
    nh.getParam("resize_w", resize_w);
    std::string adv_topic("/prometheus/camera/rgb/image_raw");
    nh.getParam("adv_topic", adv_topic);

    std::string web_cam_ip="192.168.31.64";
    if (nh.getParam("web_cam_ip", web_cam_ip))
    {
        std::cout << "set web_cam_ip is:" << web_cam_ip << std::endl;
    }

    std::string pipeline;

    // 在这里修改发布话题名称
    image_pub = it.advertise(adv_topic, 1);

    // 用系统默认驱动读取摄像头0，使用其他摄像头ID，请在这里修改

    cv::VideoCapture cap;

    if (camera_type == 1)
    {
        pipeline = gstreamer_pipeline(
            cam_w,
            cam_h,
            resize_w,
            resize_h,
            framerate,
            flip_method);
        cap.open(pipeline, cv::CAP_GSTREAMER);
    }
    else if (camera_type == 2)
    {
        cap.open(camera_id, cv::CAP_V4L2);
        cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('G', 'R', 'E', 'Y'));
        cap.set(CAP_PROP_CONVERT_RGB, 0);
    }
    else if (camera_type == 3) //VEYE 327
    {
        pipeline = gstreamer_pipeline_veye327(resize_w, resize_h);
        cap.open(pipeline, cv::CAP_GSTREAMER);
    }
    else if (camera_type == 4) //G1 Gimbal
    {
        pipeline = gstreamer_pipeline_G1(web_cam_ip, resize_w, resize_h);
        cap.open(pipeline, cv::CAP_GSTREAMER);
    }
    else
    {
        cap.open(camera_id);
        // 设置摄像头分辨率
        cap.set(CAP_PROP_FRAME_WIDTH, camera_width);
        cap.set(CAP_PROP_FRAME_HEIGHT, camera_height);
    }

    cv::Mat frame, frame_right;
    // 设置全屏
    // namedWindow("web_cam frame", CV_WINDOW_NORMAL);
    // setWindowProperty("web_cam frame", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    sensor_msgs::ImagePtr msg;

    long frame_cnt(0);
    bool cant_open(false);
    while (ros::ok())
    {
        cap >> frame;
        if (resize_w > 0 && resize_h > 0)
        {
            cv::resize(frame, frame, cv::Size(resize_w, resize_h));
        }
        if (camera_type == 2)
        {
            if (frame.cols > frame.rows * 2)
            {
                frame_right = frame.colRange(frame.cols / 2, frame.cols);
                frame = frame.colRange(0, frame.cols / 2);
            }
            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
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
            }
        }
        ros::spinOnce();
        // 按照设定的帧率延时，ros::Rate loop_rate(30)
        loop_rate.sleep();
    }

    cap.release();
}
