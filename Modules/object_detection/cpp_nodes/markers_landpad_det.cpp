/***************************************************************************************************************************
 *
 * 说明: 降落目标识别程序，降落板的尺寸为60cmX60cm
 *      1. 【订阅】图像话题 (默认来自web_cam)
 *         /prometheus/camera/rgb/image_raw
 *      2. 【发布】目标位置，发布话题见 Prometheus/Modules/msgs/msg/DetectionInfo.msg
 *         /prometheus/object_detection/landpad_det
 *      3. 【发布】检测结果的可视化图像话题
 *         /prometheus/camera/rgb/image_landpad_det
 ***************************************************************************************************************************/

#include <time.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <numeric>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// ros头文件
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// opencv头文件
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// topic 头文件
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <prometheus_msgs/DetectionInfo.h>

#include "printf_utils.h"

using namespace std;
using namespace cv;

// 输入图像
image_transport::Subscriber image_subscriber;
// 输入开关量
ros::Subscriber switch_subscriber;
// 发布目标在相机下到位姿
ros::Publisher position_pub;
// 发布识别后的图像
image_transport::Publisher landpad_pub;

//-------------VISION-----------
Mat img;
prometheus_msgs::DetectionInfo pose_now;

double calculation_time;

// 相机话题中的图像同步相关变量
int frame_width, frame_height;
cv::Mat cam_image_copy;
boost::shared_mutex mutex_image_callback;
bool image_status = false;
boost::shared_mutex mutex_image_status;

// 接收消息，允许暂停检测
bool is_suspanded = false;

// 图像接收回调函数，接收web_cam的话题，并将图像保存在cam_image_copy中
void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cam_image;

    try
    {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        PCOUT(0, RED, std::string("cv_bridge exception:") + e.what());
        return;
    }

    if (cam_image)
    {
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

void switchCallback(const std_msgs::Bool::ConstPtr &msg)
{
    is_suspanded = !(bool)msg->data;
    // cout << is_suspanded << endl;
}

// 根据id，方法该二维码的边长，landpad的原点
inline float idToPosition(int id, double id_to8_t[3], float landpad_det_len)
{

    float scale = 0;
    constexpr float big = 0.666667;
    constexpr float small = 0.133334;
    switch (id)
    {
    case 19:
    case 43:
        scale = big;
        id_to8_t[0] = 0.;
        id_to8_t[1] = 0.;
        id_to8_t[2] = 0.;
        break;
    case 1:
        id_to8_t[0] = -(landpad_det_len * big + landpad_det_len * small) / 2.;
        id_to8_t[1] = (landpad_det_len * big + landpad_det_len * small) / 2.;
        id_to8_t[2] = 0.;
        scale = small;
        break;
    case 2:
        id_to8_t[0] = -(landpad_det_len * big + landpad_det_len * small) / 2.;
        id_to8_t[1] = -(landpad_det_len * big + landpad_det_len * small) / 2.;
        id_to8_t[2] = 0.;
        scale = small;
        break;
    case 3:
        id_to8_t[0] = (landpad_det_len * big + landpad_det_len * small) / 2.;
        id_to8_t[1] = -(landpad_det_len * big + landpad_det_len * small) / 2.;
        id_to8_t[2] = 0.;
        scale = small;
        break;
    case 4:
        id_to8_t[0] = (landpad_det_len * big + landpad_det_len * small) / 2.;
        id_to8_t[1] = (landpad_det_len * big + landpad_det_len * small) / 2.;
        id_to8_t[2] = 0.;
        scale = small;
        break;
    }
    return scale;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landpad_det");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    std::string camera_topic, camera_info;
    if (nh.getParam("camera_topic", camera_topic))
    {
        PCOUT(0, GREEN, std::string("camera_topic is ") + camera_topic);
    }
    else
    {
        camera_topic = "/prometheus/camera/rgb/image_raw";
        PCOUT(0, YELLOW, std::string("didn't find parameter camera_topic, and camera_topic set to") + camera_topic);
    }

    if (nh.getParam("camera_info", camera_info))
    {
        PCOUT(0, GREEN, std::string("camera_info is ") + camera_info);
    }
    else
    {
        camera_info = "camera_param.yaml";
        PCOUT(0, YELLOW, std::string("didn't find parameter camera_info, and camera_info set to") + camera_info);
    }

    position_pub = nh.advertise<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/landpad_det", 10);

    // 接收开关话题
    switch_subscriber = nh.subscribe("/prometheus/switch/landpad_det", 10, switchCallback);

    // 接收图像的话题
    image_subscriber = it.subscribe(camera_topic.c_str(), 1, cameraCallback);
    // 发布ArUco检测结果的话题
    landpad_pub = it.advertise("/prometheus/camera/rgb/image_landpad_det", 1);

    sensor_msgs::ImagePtr msg_ellipse;

    std::string ros_path = ros::package::getPath("prometheus_detection");

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

    double landpad_det_len = camera_config["landpad_det_len"].as<double>();
    // DEBUG
    // cout << fx << " " << fy << " " << cx << " " << cy << " " << k1 << " " << k2 << " ";

    //--------------------------相机参数赋值---------------------
    // 相机内参
    Mat camera_matrix;
    camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0] = fx;
    camera_matrix.ptr<double>(0)[2] = cx;
    camera_matrix.ptr<double>(1)[1] = fy;
    camera_matrix.ptr<double>(1)[2] = cy;
    camera_matrix.ptr<double>(2)[2] = 1.0f;
    // 相机畸变参数k1 k2 p1 p2 k3
    Mat distortion_coefficients;
    distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
    distortion_coefficients.ptr<double>(0)[0] = k1;
    distortion_coefficients.ptr<double>(1)[0] = k2;
    distortion_coefficients.ptr<double>(2)[0] = p1;
    distortion_coefficients.ptr<double>(3)[0] = p2;
    distortion_coefficients.ptr<double>(4)[0] = k3;

    // ArUco Marker字典选择以及旋转向量和评议向量初始化
    Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // 旋转向量，转移向量
    vector<double> rv(3), tv(3);
    // 旋转向量，转移向量
    cv::Mat rvec(rv), tvec(tv);
    // cv::VideoCapture capture(0);
    float last_x(0), last_y(0), last_z(0), last_yaw(0), last_az(0), last_ay(0), last_ax(0), last_qx(0), last_qy(0), last_qz(0), last_qw(0);
    // 切换标志位，是否检测
    bool switch_state = is_suspanded;

    // 节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate loopRate(20);
    //----------------------------------------主循环------------------------------------
    // const auto wait_duration = std::chrono::milliseconds(2000);
    while (ros::ok())
    {
        ros::spinOnce();
        if (!getImageStatus() && ros::ok())
        {
            PCOUT(-1, WHITE, "Waiting for image");
            continue;
        }
        PCOUT(-1, GREEN, "RUNING ...");

        if (switch_state != is_suspanded)
        {
            switch_state = is_suspanded;
            if (!is_suspanded)
            {
                PCOUT(0, WHITE, "Start Detection");
            }
            else
            {
                PCOUT(0, WHITE, "Stop Detection");
            }
        }

        if (!is_suspanded)
        {
            {
                boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
                img = cam_image_copy.clone();
            }

            clock_t start = clock();

            //------------------调用ArUco Marker库对图像进行识别--------------
            // markerids存储每个识别到二维码的编号  markerCorners每个二维码对应的四个角点的像素坐标
            std::vector<int> markerids, markerids_deted;
            // markerids_deted 二维码原点
            vector<vector<Point2f>> markerCorners, markerCorners_deted, rejectedCandidate;

            Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
            cv::aruco::detectMarkers(img, dictionary, markerCorners_deted, markerids_deted, parameters, rejectedCandidate);

            // 如果检测到二维码，按顺序进行检测
            // 19中间最大， 43中间最小， 1左上角， 2右上角， 3左下角， 4右下角
            int landpad_id[6]{19, 43, 1, 2, 3, 4};
            for (auto i = std::begin(landpad_id); i != std::end(landpad_id); ++i)
            {
                for (int ii = 0; ii < markerids_deted.size(); ++ii)
                {
                    if (markerids_deted[ii] == *i)
                    {
                        markerCorners.push_back(markerCorners_deted[ii]);
                        markerids.push_back(*i);
                        break;
                    }
                }
                if (!markerids.empty())
                    break;
            }
            // 不同到id有着不同的大小参数
            if (!markerids.empty())
            {
                // 可视化
                aruco::drawDetectedMarkers(img, markerCorners, markerids);
                double id_to8_t[3];
                float scale = idToPosition(markerids[0], id_to8_t, landpad_det_len);
                // std::cout << markerids[0] << " >> " <<  id_to8_t[0] << " " <<  id_to8_t[1] << " " << id_to8_t[2] << std::endl;

                vector<Vec3d> rvecs, tvecs;
                aruco::estimatePoseSingleMarkers(markerCorners, landpad_det_len * scale, camera_matrix, distortion_coefficients, rvecs, tvecs);
                aruco::drawAxis(img, camera_matrix, distortion_coefficients, rvecs[0], tvecs[0], landpad_det_len * scale * 0.5f);

                // 姿态
                cv::Mat rotation_matrix;
                // 相机姿态: 旋转向量 -> 旋转矩阵
                cv::Rodrigues(rvecs[0], rotation_matrix);
                Eigen::Matrix3d rotation_matrix_eigen;
                cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
                Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix_eigen);
                q.normalize(); //标准化

                // 位置
                std::vector<double> vec_t{tvecs[0][0], tvecs[0][1], tvecs[0][2]};
                cv::Mat vec_t_mat{vec_t};
                vec_t_mat = vec_t_mat;
                vec_t_mat.convertTo(vec_t_mat, CV_32FC1);

                // 原点位置
                cv::Mat id_to8_t_mat = cv::Mat(3, 1, CV_32FC1, id_to8_t);
                rotation_matrix.convertTo(rotation_matrix, CV_32FC1);
                cv::Mat id_8_t = rotation_matrix * id_to8_t_mat + vec_t_mat;
                // 整合数据发布

                float o_tx = id_8_t.at<float>(0);
                float o_ty = id_8_t.at<float>(1);
                float o_tz = id_8_t.at<float>(2);

                // 将解算的位置转化成旋转矩阵 并旋转计算无人机相对于目标的位置
                float r11 = rotation_matrix.ptr<float>(0)[0];
                float r12 = rotation_matrix.ptr<float>(0)[1];
                float r13 = rotation_matrix.ptr<float>(0)[2];
                float r21 = rotation_matrix.ptr<float>(1)[0];
                float r22 = rotation_matrix.ptr<float>(1)[1];
                float r23 = rotation_matrix.ptr<float>(1)[2];
                float r31 = rotation_matrix.ptr<float>(2)[0];
                float r32 = rotation_matrix.ptr<float>(2)[1];
                float r33 = rotation_matrix.ptr<float>(2)[2];

                // 计算欧拉角; 旋转矩阵转换为欧拉角
                float thetaz = atan2(r21, r11) / CV_PI * 180;
                float thetay = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / CV_PI * 180;
                float thetax = atan2(r32, r33) / CV_PI * 180;

                // C2W代表 相机坐标系转换到世界坐标系姿态变化   W2C代表 世界坐标系转换到相机坐标系 Theta为欧拉角
                cv::Point3f Theta_C2W;      // 相机 -> 世界
                cv::Point3f Theta_W2C;      // 世界 -> 相机
                cv::Point3f Position_OcInW; // 相机坐标系下的位置

                Theta_C2W.z = thetaz;
                Theta_C2W.y = thetay;
                Theta_C2W.x = thetax;

                Theta_W2C.x = -1 * thetax;
                Theta_W2C.y = -1 * thetay;
                Theta_W2C.z = -1 * thetaz;

                Eigen::Vector3d eulerVec;
                eulerVec(0) = (Theta_C2W.z) / 180 * CV_PI;
                double A1_yaw = eulerVec(0);

                // 将解算后的位置发给控制端
                pose_now.header.stamp = ros::Time::now();
                pose_now.detected = true;
                pose_now.frame = 0;
                // 目标点位置
                pose_now.position[0] = o_tx;
                pose_now.position[1] = o_ty;
                pose_now.position[2] = o_tz;
                pose_now.attitude[0] = thetaz;
                pose_now.attitude[1] = thetay;
                pose_now.attitude[2] = thetax;
                pose_now.attitude_q[0] = q.x();
                pose_now.attitude_q[1] = q.y();
                pose_now.attitude_q[2] = q.z();
                pose_now.attitude_q[3] = q.w();
                // 视线角，球体坐标系
                pose_now.sight_angle[0] = atan(o_tx / o_tz);
                pose_now.sight_angle[1] = atan(o_ty / o_tz);
                // 偏航角误差
                pose_now.yaw_error = A1_yaw;

                // 记录历史值
                last_x = pose_now.position[0];
                last_y = pose_now.position[1];
                last_z = pose_now.position[2];
                last_az = pose_now.attitude[0];
                last_ay = pose_now.attitude[1];
                last_ax = pose_now.attitude[2];
                last_qx = pose_now.attitude_q[0];
                last_qy = pose_now.attitude_q[1];
                last_qz = pose_now.attitude_q[2];
                last_qw = pose_now.attitude_q[3];
                last_yaw = pose_now.yaw_error;
            }
            // 找不到就使用历史值
            else
            {
                pose_now.header.stamp = ros::Time::now();
                pose_now.detected = false;
                pose_now.frame = 0;
                pose_now.position[0] = last_x;
                pose_now.position[1] = last_y;
                pose_now.position[2] = last_z;
                pose_now.attitude[0] = last_az;
                pose_now.attitude[1] = last_ay;
                pose_now.attitude[2] = last_ax;
                pose_now.attitude_q[0] = last_qx;
                pose_now.attitude_q[1] = last_qy;
                pose_now.attitude_q[2] = last_qz;
                pose_now.attitude_q[3] = last_qw;
                pose_now.sight_angle[0] = atan(last_x / last_z);
                pose_now.sight_angle[1] = atan(last_y / last_z);
                pose_now.yaw_error = last_yaw;
            }
            position_pub.publish(pose_now);

            // 计算算法运行时间
            clock_t finish = clock();
            calculation_time = (finish - start) / 1000;

            msg_ellipse = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            landpad_pub.publish(msg_ellipse);
        }

        // cv::imshow("test",img);
        // cv::waitKey(1);
        loopRate.sleep();
    }
}
