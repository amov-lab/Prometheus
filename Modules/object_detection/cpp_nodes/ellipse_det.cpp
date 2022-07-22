#include "ellipse_detector.h"
#include "printf_utils.h"
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <prometheus_msgs/MultiDetectionInfo.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <shared_mutex>
#include <std_msgs/Float32.h>

std::shared_mutex g_mutex;
int frame_width = 640, frame_height = 480;

cv::Mat g_camera_matrix, g_dist_coeffs;
cv::Mat g_frame;
void imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (cam_image)
    {
        std::unique_lock lock(g_mutex);
        cv::Mat tmp = cam_image->image.clone();
        // NOTE: 图像去畸变
        // g_frame = tmp
        cv::undistort(tmp, g_frame, g_camera_matrix, g_dist_coeffs);
        frame_width = cam_image->image.size().width;
        frame_height = cam_image->image.size().height;
    }
}

float g_camera_height = 0;
void heightCb(const std_msgs::Float32ConstPtr &msg)
{
    g_camera_height = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ellipse_det");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    std::string input_image_topic, output_image_topic, camera_params, det_info_topic, camera_height_topic;
    nh.param<std::string>("input_image_topic", input_image_topic, "/prometheus/camera/rgb/image_raw");
    nh.param<std::string>("output_image_topic", output_image_topic, "/prometheus/detection/image_raw");
    nh.param<std::string>("camera_height_topic", camera_height_topic, "/camera/height");
    nh.param<std::string>("camera_params", camera_params, "");
    nh.param<std::string>("det_info_topic", det_info_topic, "/prometheus/ellipse_det");

    if (camera_params.empty() || access(camera_params.c_str(), 0x00) != 0)
    {
        throw camera_params + " path does not exist or not provided";
        return -1;
    }
    cv::FileStorage fs(camera_params, cv::FileStorage::READ);
    fs["camera_matrix"] >> g_camera_matrix;
    fs["distortion_coefficients"] >> g_dist_coeffs;

    // 订阅图像话题
    image_transport::Subscriber image_subscriber = it.subscribe(input_image_topic, 1, imageCb);
    // 发布检测可视化图像话题
    image_transport::Publisher image_pub = it.advertise(output_image_topic, 1);

    // 发布MultiDetectionInfo话题
    ros::Publisher det_info_pub = nh.advertise<prometheus_msgs::MultiDetectionInfo>(det_info_topic, 1);

    // 圆检测参数调整
    EllipseDetector ellipse_detector;
    float fMaxCenterDistance = sqrt(float(width * width + height * height)) * 0.05f;
    ellipse_detector.SetParameters(cv::Size(5, 5), 1.306, 1.f, fMaxCenterDistance, 9, 2.984, 0.111, 0.511, 0.470, 22, 0.946, 0.121, 0.468, 0.560, 0.202);

    ros::Rate rate(60);
    while (ros::ok())
    {
        prometheus_msgs::MultiDetectionInfo det_info;
        det_info.detect_or_track = 0;
        ros::spinOnce();
        if (g_frame.empty())
        {
            PCOUT(-1, YELLOW, "wait for get image data");
            continue;
        }
        cv::Mat frame;
        {
            std::unique_lock lock(g_mutex);
            frame = g_frame.clone();
        }
        std::vector<Ellipse> ellsCned;
        double t0 = (double)cv::getTickCount();
        ellipse_detector.Detect(frame, ellsCned);

        sensor_msgs::ImagePtr det_output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        // cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
        det_info.num_objs = ellsCned.size();
        for (int i = 0; i < ellsCned.size(); i++)
        {
            const Ellipse &ellipse = ellsCned[i];
            prometheus_msgs::DetectionInfo info;
            // NOTE: 如果区分无人机起点和靶标?
            cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8U);
            cv::ellipse(mask, cv::Point(cvRound(ellipse.xc_), cvRound(ellipse.yc_)), cv::Size(cvRound(ellipse.a_), cvRound(ellipse.b_)), ellipse.rad_ * 180 / CV_PI, 0, 360, cv::Scalar(255, 255, 255), -1);
            cv::Mat tmp;
            frame.copyTo(tmp, mask);
            // cv::bitwise_and(frame, cv::noArray(), tmp, mask);
            cv::inRange(tmp, cv::Scalar(150, 150, 150), cv::Scalar(255, 255, 255), tmp);

            int *tmpi = nullptr;
            if (static_cast<float>(cv::countNonZero(tmp)) / cv::countNonZero(mask) > 0.1)
            {
                info.object_name = "S";
                cv::Size sz = cv::getTextSize(info.object_name, 0, 0.8, 1, tmpi);
                cv::putText(frame, info.object_name, cv::Point(ellipse.xc_ - sz.width / 2, ellipse.yc_ + sz.height / 2), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
            }
            else
            {
                info.object_name = "T";
                cv::Size sz = cv::getTextSize(info.object_name, 0, 0.8, 1, tmpi);
                cv::putText(frame, info.object_name, cv::Point(ellipse.xc_ - sz.width / 2, ellipse.yc_ + sz.height / 2), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
            }
            info.frame = 0;
            info.detected = true;
            // NOTE: 目标位置估计
            info.position[0] = g_camera_height * (ellipse.yc_ - frame_height / 2) / g_camera_matrix.at<float>(1, 1);
            info.position[1] = g_camera_height * (ellipse.xc_ - frame_width / 2) / g_camera_matrix.at<float>(0, 0);
            info.position[2] = g_camera_height;
            info.sight_angle[0] = (ellipse.xc_ - frame_width / 2) / (frame_width / 2) * std::atan(frame_width / (2 * g_camera_matrix.at<float>(0, 0)));
            info.sight_angle[1] = (ellipse.yc_ - frame_height / 2) / (frame_height / 2) * std::atan(frame_height / (2 * g_camera_matrix.at<float>(1, 1)));
            info.pixel_position[0] = ellipse.xc_;
            info.pixel_position[1] = ellipse.yc_;
            det_info.detection_infos.push_back(info);
        }

        double dt = ((double)cv::getTickCount() - t0) / cv::getTickFrequency();
        char msg[256];
        sprintf(msg, "FPS: %f", 1.f / dt);
        cv::putText(frame, msg, cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, 4, 0);

        ellipse_detector.DrawDetectedEllipses(frame, ellsCned, 1, 6);
        image_pub.publish(det_output_msg);
        cv::imshow("show", frame);
        cv::waitKey(1);

        det_info_pub.publish(det_info);
        rate.sleep();
    }
    return 0;
}