/***************************************************************************************************************************
 * landpad_det.cpp
 * Author: Jario
 * Update Time: 2020.1.12
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
#include <prometheus_msgs/Message.h>

#include "message_utils.h"

using namespace std;
using namespace cv;

double threshold_error=0.4;


//---------------------------variables---------------------------------------
//------------ROS TOPIC---------
//【订阅】无人机位置
ros::Subscriber drone_pose_sub;
//【订阅】小车位置
ros::Subscriber vehicle_pose_sub;
//【订阅】输入图像
image_transport::Subscriber image_subscriber;
//【订阅】输入开关量
ros::Subscriber switch_subscriber;
//【发布】无人机和小车相对位置
ros::Publisher position_pub;
//【发布】识别后的图像
image_transport::Publisher landpad_pub;
//【发布】调试消息
ros::Publisher message_pub;
std::string msg_node_name;

//-------------VISION-----------
Mat img;
prometheus_msgs::DetectionInfo pose_now;

//-------------TIME-------------
ros::Time begin_time;
float photo_time;
double calculation_time;

// 相机话题中的图像同步相关变量
int frame_width, frame_height;
std_msgs::Header image_header;
cv::Mat cam_image_copy;
boost::shared_mutex mutex_image_callback;
bool image_status = false;
boost::shared_mutex mutex_image_status;

// 无人机位姿message
geometry_msgs::Pose pos_drone_optitrack;
Eigen::Vector3d euler_drone_optitrack;
Eigen::Quaterniond q_drone;
// 小车位姿message
geometry_msgs::Pose pos_vehicle_optitrack;
Eigen::Vector3d euler_vehicle_optitrack;
Eigen::Quaterniond q_vehicle;

// 保存的上次观测的位置 用于cluster算法使用
Eigen::Vector3d last_position;
bool bool_last_position = false;
// 接收消息，允许暂停检测
bool is_suspanded = false;
bool local_print = false;
bool message_print = true;

void printf_result();


//-----------------利用Euler角进行三次旋转得到无人机相对目标的位置------------------
void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
    double x1 = x;  // 将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
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
    double y1 = y;  // 将变量拷贝一次，保证&y == &y这种情况下也能计算正确
    double z1 = z;
    double rx = thetax * CV_PI / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}
// 四元数转Euler
// q0 q1 q2 q3
// w x y z
void quaternion_2_euler(Eigen::Quaterniond quat, Eigen::Vector3d &angle)
{
    angle(0) = atan2(2.0 * (quat.z() * quat.y() + quat.w() * quat.x()), 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
    angle(1) = asin(2.0 * (quat.y() * quat.w() - quat.z() * quat.x()));
    // angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    angle(2) = atan2(2.0 * (quat.z() * quat.w() + quat.x() * quat.y()), 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
}

//--------------------------利用optitrack获取真值-------------------------------

// 获取系统时间
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}



// 图像接收回调函数，接收web_cam的话题，并将图像保存在cam_image_copy中
void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (local_print)
        ROS_DEBUG("[LandpadDetector] USB image received.");

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
    ros::init(argc, argv, "landpad_det");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // 发布调试消息
    msg_node_name = "/prometheus/message/landpad_det";
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

    position_pub = nh.advertise<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/landpad_det", 10);


    // 接收开关话题
    switch_subscriber = nh.subscribe("/prometheus/switch/landpad_det", 10, switchCallback);

    // 接收图像的话题
    image_subscriber = it.subscribe(camera_topic.c_str(), 1, cameraCallback);
    // 发布ArUco检测结果的话题
    landpad_pub = it.advertise("/prometheus/camera/rgb/image_landpad_det", 1);
    

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

    double landpad_det_len = camera_config["landpad_det_len"].as<double>();
    // DEBUG
    // cout << fx << " " << fy << " " << cx << " " << cy << " " << k1 << " " << k2 << " ";

    //--------------------------相机参数赋值---------------------
    // 相机内参
    Mat camera_matrix;
    camera_matrix = cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0] = fx;
    camera_matrix.ptr<double>(0)[2] = cx;
    camera_matrix.ptr<double>(1)[1] = fy;
    camera_matrix.ptr<double>(1)[2] = cy;
    camera_matrix.ptr<double>(2)[2] = 1.0f;
    // 相机畸变参数k1 k2 p1 p2 k3
    Mat distortion_coefficients;
    distortion_coefficients = cv::Mat(5,1,CV_64FC1,cv::Scalar::all(0));
    distortion_coefficients.ptr<double>(0)[0] = k1;
    distortion_coefficients.ptr<double>(1)[0] = k2;
    distortion_coefficients.ptr<double>(2)[0] = p1;
    distortion_coefficients.ptr<double>(3)[0] = p2;
    distortion_coefficients.ptr<double>(4)[0] = k3;

    // ArUco Marker字典选择以及旋转向量和评议向量初始化
    Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(10);

    vector<double> rv(3), tv(3);
    cv::Mat rvec(rv), tvec(tv);
    // cv::VideoCapture capture(0);
    float last_x(0), last_y(0), last_z(0), last_yaw(0), last_az(0), last_ay(0), last_ax(0), last_qx(0), last_qy(0), last_qz(0), last_qw(0);
    bool switch_state = is_suspanded;

    // 节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate loopRate(20);
    ros::Rate loopRate_1Hz(1);
    //----------------------------------------主循环------------------------------------
    // const auto wait_duration = std::chrono::milliseconds(2000);
    while (ros::ok())
    {
        while (!getImageStatus() && ros::ok()) 
        {
            if (local_print)
                cout << "Waiting for image." << endl;
            if (message_print)
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "Waiting for image.");
            
            // std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
            loopRate_1Hz.sleep();
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
            img = cam_image_copy.clone();
        }

        clock_t start = clock();

        //------------------调用ArUco Marker库对图像进行识别--------------
        // markerids存储每个识别到二维码的编号  markerCorners每个二维码对应的四个角点的像素坐标
        std::vector<int> markerids, markerids_deted;
        vector<vector<Point2f> > markerCorners, markerCorners_deted, rejectedCandidate;

        Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(img, dictionary, markerCorners_deted, markerids_deted, parameters, rejectedCandidate);

        if (markerids_deted.size() > 0)
        {
            for (int tt=0; tt<markerids_deted.size(); tt++) {
                if (19 == markerids_deted[tt]) {
                    markerids.push_back(markerids_deted[tt]);
                    markerCorners.push_back(markerCorners_deted[tt]);
                }
            }
            if (markerids.size() == 0) {
                for (int tt=0; tt<markerids_deted.size(); tt++) {
                    if (43 == markerids_deted[tt]) {
                        markerids.push_back(markerids_deted[tt]);
                        markerCorners.push_back(markerCorners_deted[tt]);
                    }
                }
            }
            if (markerids.size() == 0) {
                for (int tt=0; tt<markerids_deted.size(); tt++) {
                    if (1 == markerids_deted[tt]) {
                        markerids.push_back(markerids_deted[tt]);
                        markerCorners.push_back(markerCorners_deted[tt]);
                    }
                }
            }
            if (markerids.size() == 0) {
                for (int tt=0; tt<markerids_deted.size(); tt++) {
                    if (2 == markerids_deted[tt]) {
                        markerids.push_back(markerids_deted[tt]);
                        markerCorners.push_back(markerCorners_deted[tt]);
                    }
                }
            }
            if (markerids.size() == 0) {
                for (int tt=0; tt<markerids_deted.size(); tt++) {
                    if (3 == markerids_deted[tt]) {
                        markerids.push_back(markerids_deted[tt]);
                        markerCorners.push_back(markerCorners_deted[tt]);
                    }
                }
            }
            if (markerids.size() == 0) {
                for (int tt=0; tt<markerids_deted.size(); tt++) {
                    if (4 == markerids_deted[tt]) {
                        markerids.push_back(markerids_deted[tt]);
                        markerCorners.push_back(markerCorners_deted[tt]);
                    }
                }
            }
        }

        //-------------------多于一个目标被识别到，进入算法-----------------
        if (markerids.size() > 0)
        {
            aruco::drawDetectedMarkers(img, markerCorners, markerids);

            std::vector<float> collected_tx, collected_ty, collected_tz;
            std::vector<float> collected_qx, collected_qy, collected_qz, collected_qw;


            int tt=0;
            vector< Vec3d > rvecs, tvecs;
            if (19 == markerids[tt])
            {
                vector<vector<Point2f> > markerCornersONE;
                markerCornersONE.push_back(markerCorners[tt]);
                aruco::estimatePoseSingleMarkers(markerCornersONE, landpad_det_len*0.666667, camera_matrix, distortion_coefficients, rvecs, tvecs);
                aruco::drawAxis(img, camera_matrix, distortion_coefficients, rvecs[0], tvecs[0],landpad_det_len*0.666667*0.5f);
            }
            else if (43 == markerids[tt])
            {
                vector<vector<Point2f> > markerCornersONE;
                markerCornersONE.push_back(markerCorners[tt]);
                aruco::estimatePoseSingleMarkers(markerCornersONE, landpad_det_len*0.066667, camera_matrix, distortion_coefficients, rvecs, tvecs);
                aruco::drawAxis(img, camera_matrix, distortion_coefficients, rvecs[0], tvecs[0],landpad_det_len*0.066667*0.5f);
            }
            else if (1 == markerids[tt] || 2 == markerids[tt] || 3 == markerids[tt] || 4 == markerids[tt])
            {
                vector<vector<Point2f> > markerCornersONE;
                markerCornersONE.push_back(markerCorners[tt]);
                aruco::estimatePoseSingleMarkers(markerCornersONE, landpad_det_len*0.133334, camera_matrix, distortion_coefficients, rvecs, tvecs);
                aruco::drawAxis(img, camera_matrix, distortion_coefficients, rvecs[0], tvecs[0],landpad_det_len*0.133334*0.5f);
            }

            cv::Mat rotation_matrix;
            cv::Rodrigues(rvecs[0], rotation_matrix);
            Eigen::Matrix3d rotation_matrix_eigen;
            cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
            Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix_eigen);
            q.normalize();

            /*if (43 != markerids[tt] && 19 != markerids[tt]) {
                static tf::TransformBroadcaster br;
                tf::Transform world2camera = tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(tvecs[0][0], tvecs[0][1], tvecs[0][2]));
                char obj_str[16];
                sprintf(obj_str, "object-%d", markerids[tt]);
                tf::StampedTransform trans_world2camera = tf::StampedTransform(world2camera, ros::Time(), "camera", obj_str);
                br.sendTransform(trans_world2camera);
            }*/

            std::vector<double> vec_t{tvecs[0][0], tvecs[0][1], tvecs[0][2]};
            cv::Mat vec_t_mat{vec_t};
            vec_t_mat = vec_t_mat;
            vec_t_mat.convertTo(vec_t_mat, CV_32FC1);
            // cout << "vec_t_mat.size():" << vec_t_mat.size() << endl;
            // cout << "vec_t_mat.type():" << vec_t_mat.type() <<endl;
            std::vector<double> id_to8_t(3);

            if (19 == markerids[tt] || 43 == markerids[tt])
            {
                id_to8_t[0] = 0.;
                id_to8_t[1] = 0.;
                id_to8_t[2] = 0.;
            }
            else if (1 == markerids[tt])
            {
                id_to8_t[0] = -(landpad_det_len*0.666667 + landpad_det_len*0.133334) / 2.;
                id_to8_t[1] = (landpad_det_len*0.666667 + landpad_det_len*0.133334) / 2.;
                id_to8_t[2] = 0.;
            }
            else if (2 == markerids[tt])
            {
                id_to8_t[0] = -(landpad_det_len*0.666667 + landpad_det_len*0.133334) / 2.;
                id_to8_t[1] = -(landpad_det_len*0.666667 + landpad_det_len*0.133334) / 2.;
                id_to8_t[2] = 0.;
            }
            else if (3 == markerids[tt])
            {
                id_to8_t[0] = (landpad_det_len*0.666667 + landpad_det_len*0.133334) / 2.;
                id_to8_t[1] = -(landpad_det_len*0.666667 + landpad_det_len*0.133334) / 2.;
                id_to8_t[2] = 0.;
            }
            else if (4 == markerids[tt])
            {
                id_to8_t[0] = (landpad_det_len*0.666667 + landpad_det_len*0.133334) / 2.;
                id_to8_t[1] = (landpad_det_len*0.666667 + landpad_det_len*0.133334) / 2.;
                id_to8_t[2] = 0.;
            }

            cv::Mat id_to8_t_mat{id_to8_t};
            id_to8_t_mat.convertTo(id_to8_t_mat, CV_32FC1);

            rotation_matrix.convertTo(rotation_matrix, CV_32FC1);
            // cv::invert(rotation_matrix, rotation_matrix);
            cv::Mat id_8_t = rotation_matrix * id_to8_t_mat + vec_t_mat;
            // cout << id_8_t << endl;

            float o_tx = id_8_t.at<float>(0);
            float o_ty = id_8_t.at<float>(1);
            float o_tz = id_8_t.at<float>(2);

            float o_qx = q.x();
            float o_qy = q.y();
            float o_qz = q.z();
            float o_qw = q.w();


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

            // 计算欧拉角
            float thetaz = atan2(r21, r11) / CV_PI * 180;
            float thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
            float thetax = atan2(r32, r33) / CV_PI * 180;

            // C2W代表 相机坐标系转换到世界坐标系  W2C代表 世界坐标系转换到相机坐标系 Theta为欧拉角
            cv::Point3f Theta_C2W;
            cv::Point3f Theta_W2C;
            cv::Point3f Position_OcInW;

            Theta_C2W.z = thetaz;
            Theta_C2W.y = thetay;
            Theta_C2W.x = thetax;

            Theta_W2C.x = -1 * thetax;
            Theta_W2C.y = -1 * thetay;
            Theta_W2C.z = -1 * thetaz;

            Position_OcInW.x = id_8_t.at<float>(0);
            Position_OcInW.y = id_8_t.at<float>(1);
            Position_OcInW.z = id_8_t.at<float>(2);

            Eigen::Vector3d eulerVec;
            eulerVec(0) = (Theta_C2W.z) / 180 * CV_PI;
            double A1_yaw = eulerVec(0);

            // 将解算后的位置发给控制端
            pose_now.header.stamp = ros::Time::now();
            pose_now.detected = true;
            pose_now.frame = 0;
            pose_now.position[0] = o_tx;
            pose_now.position[1] = o_ty;
            pose_now.position[2] = o_tz;
            pose_now.attitude[0] = thetaz;
            pose_now.attitude[1] = thetay;
            pose_now.attitude[2] = thetax;
            pose_now.attitude_q[0] = o_qx;
            pose_now.attitude_q[1] = o_qy;
            pose_now.attitude_q[2] = o_qz;
            pose_now.attitude_q[3] = o_qw;
            pose_now.sight_angle[0] = atan(o_tx / o_tz);
            pose_now.sight_angle[1] = atan(o_ty / o_tz);
            pose_now.yaw_error = A1_yaw;

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
        clock_t finish=clock();
        calculation_time=(finish-start)/1000;
        
        // 打印
        // printf_result();

        // 画出识别到的二维码
        // cv::aruco::drawDetectedMarkers(img, markerCorners, markerids);
        
        msg_ellipse = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        landpad_pub.publish(msg_ellipse);
        }

        // cv::imshow("test",img);
        ros::spinOnce();
        cv::waitKey(1);
        loopRate.sleep();
    }
}


void printf_result()
{
    // 固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout<<setprecision(4);
    // 左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Landpad Detection<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(pose_now.detected)
    {
        cout << "is_detected: ture" <<endl;
    }else
    {
        cout << "is_detected: false" <<endl;
    }
    cout << "pos_target: [X Y Z] : " << pose_now.position[0]  << " [m] "<< pose_now.position[1] <<" [m] " << pose_now.position[2] << " [m] "<<endl;
    cout << "pos_target: [Yaw] :   " << pose_now.yaw_error/3.1415926 *180    << " [du] "<<endl;
    cout << "calculation_time =    " << time << " [ms] " << endl;
}
