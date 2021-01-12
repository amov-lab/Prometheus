/***************************************************************************************************************************
 * aruco_det_v2.cpp
 * Author: Jario
 * Update Time: 2020.12.11
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
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <image_transport/image_transport.h>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>  
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

// #include "message_utils.h"


using namespace std;
using namespace cv;

//【订阅】运行状态转换开关
ros::Subscriber switch_subscriber;
//【订阅】输入图像
image_transport::Subscriber image_subscriber;
//【发布】检测得到的位置与姿态信息
ros::Publisher pose_pub;
//【发布】检测结果图像
image_transport::Publisher aruco_pub;


// 使用cout打印消息
bool local_print = true;


// 相机话题中的图像同步相关变量
int frame_width, frame_height;
std_msgs::Header image_header;
cv::Mat cam_image_copy;
boost::shared_mutex mutex_image_callback;
bool image_status = false;
boost::shared_mutex mutex_image_status;

// 运行状态
// 0: 正常检测Aruco码，输出位姿
// 1: 世界坐标系标定，标定后，检测结果被转换到世界坐标系下
int run_state(0);


void switchCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("receiving [%s]", msg->data.c_str());
    if ("calibrate" == msg->data)
    {
        ROS_INFO("run_state = 1");
        run_state = 1;
    }
}


// 图像接收回调函数，接收web_cam的话题，并将图像保存在cam_image_copy中
void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (local_print) ROS_DEBUG("[ArucoDetector] USB image received.");

    cv_bridge::CvImagePtr cam_image;

    try {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_header = msg->header;
    } catch (cv_bridge::Exception& e) {
        if (local_print) ROS_ERROR("cv_bridge exception: %s", e.what());
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


static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_det_v2");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    // 更新频率为60HZ
    ros::Rate loop_rate(60);

    std::string camera_topic = "/prometheus/camera/rgb/image_raw";
    std::string camera_params_yaml;
    std::string output_topic = "/prometheus/camera/rgb/image_aruco_det";

    int dictionaryId(2);
    float markerLength(0.0215);

    if (nh.getParam("camera_topic", camera_topic)) {
        if (local_print) ROS_INFO("camera_topic is %s", camera_topic.c_str());
    } else {
        if (local_print) ROS_WARN("didn't find parameter camera_topic");
    }
    if (nh.getParam("camera_parameters", camera_params_yaml)) {
        if (local_print) ROS_INFO("camera_parameters is %s", camera_params_yaml.c_str());
    } else {
        if (local_print) ROS_WARN("didn't find camera_parameters");
    }
    if (nh.getParam("output_topic", output_topic)) {
        if (local_print) ROS_INFO("output_topic is %s", output_topic.c_str());
    } else {
        if (local_print) ROS_WARN("didn't find parameter output_topic");
    }

    if (nh.getParam("dictionary_type", dictionaryId)) {
        if (local_print) ROS_INFO("dictionary_type is %d", dictionaryId);
    } else {
        if (local_print) ROS_WARN("didn't find parameter dictionary_type");
    }
    if (nh.getParam("marker_length", markerLength)) {
        if (local_print) ROS_INFO("marker_length is %f", markerLength);
    } else {
        if (local_print) ROS_WARN("didn't find parameter marker_length");
    }

    //【订阅】运行状态转换开关
    switch_subscriber = nh.subscribe("/prometheus/object_detection/aruco_navigation_switch", 1, switchCallback);
    //【订阅】输入图像
    image_subscriber = it.subscribe(camera_topic.c_str(), 1, cameraCallback);

    //【发布】检测得到的位置与姿态信息
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/prometheus/object_detection/aruco_det_v2", 1);
    //【发布】检测结果图像
    aruco_pub = it.advertise(output_topic.c_str(), 1);

    std::string ros_path = ros::package::getPath("prometheus_detection");
    if (local_print) ROS_INFO("DETECTION_PATH: %s", ros_path.c_str());

    cv::Mat camMatrix, distCoeffs;
    bool readOk = readCameraParameters(camera_params_yaml.c_str(), camMatrix, distCoeffs);
    if (!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
    }

    if (local_print) {
        cout << "[camMatrix]:" << endl;
        cout << camMatrix << endl;
        cout << "[distCoeffs]:" << endl;
        cout << distCoeffs << endl;
    }


    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

    cv::Mat frame, frameCopy;
    const auto wait_duration = std::chrono::milliseconds(1000);
    while (ros::ok())
	{
        while (!getImageStatus() && ros::ok()) 
        {
            if (local_print) cout << "Waiting for image." << endl;
            std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
        }

        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            frame = cam_image_copy.clone();
        }

        if (!frame.empty())
        {
            vector< int > ids;
            vector< vector< Point2f > > corners, rejected;
            vector< Vec3d > rvecs, tvecs;

            // detect markers and estimate pose
            aruco::detectMarkers(frame, dictionary, corners, ids, detectorParams, rejected);
            if (ids.size() > 0)
                aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);

            frame.copyTo(frameCopy);
            if(ids.size() > 0) {
                aruco::drawDetectedMarkers(frameCopy, corners, ids);

                for(unsigned int i = 0; i < ids.size(); i++) {
                    aruco::drawAxis(frameCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvecs[i], rotation_matrix);
                    Eigen::Matrix3d rotation_matrix_eigen;
                    cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
                    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix_eigen);
                    q.normalize();

                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = "camera";
                    pose.pose.position.x = tvecs[i][0];
                    pose.pose.position.y = tvecs[i][1];
                    pose.pose.position.z = tvecs[i][2];
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();
                    pose_pub.publish(pose);

                    static tf::TransformBroadcaster br;
                    tf::Transform world2camera = tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]));
                    char obj_str[16];
                    sprintf(obj_str, "object-%d", ids[i]);
                    tf::StampedTransform trans_world2camera = tf::StampedTransform(world2camera, ros::Time(pose.header.stamp), "camera", obj_str);
                    br.sendTransform(trans_world2camera);

                    if (1 == run_state)
                    {
                        std::vector<cv::Point3f> collected_pt3f;
                        if (ids[i] >= 0 && ids[i] <= 16)
                        {
                            std::vector<double> vec_t{tvecs[i][0], tvecs[i][1], tvecs[i][2]};
	                        cv::Mat vec_t_mat{vec_t};
                            vec_t_mat = vec_t_mat;
                            vec_t_mat.convertTo(vec_t_mat, CV_32FC1);
                            //cout << "vec_t_mat.size():" << vec_t_mat.size() << endl;
                            //cout << "vec_t_mat.type():" << vec_t_mat.type() <<endl;
                            std::vector<double> id_to8_t(3);
                            if (ids[i] == 0)
                            {
                                id_to8_t[0] = 0.0345;
                                id_to8_t[1] = -0.0345*3;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 1)
                            {
                                id_to8_t[0] = -0.0345;
                                id_to8_t[1] = -0.0345*3;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 2)
                            {
                                id_to8_t[0] = 0.0345*2;
                                id_to8_t[1] = -0.0345*2;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 3)
                            {
                                id_to8_t[0] = 0.;
                                id_to8_t[1] = -0.0345*2;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 4)
                            {
                                id_to8_t[0] = -0.0345*2;
                                id_to8_t[1] = -0.0345*2;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 5)
                            {
                                id_to8_t[0] = 0.0345;
                                id_to8_t[1] = -0.0345;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 6)
                            {
                                id_to8_t[0] = -0.0345;
                                id_to8_t[1] = -0.0345;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 7)
                            {
                                id_to8_t[0] = 0.0345*2;
                                id_to8_t[1] = 0.;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 8)
                            {
                                collected_pt3f.push_back(cv::Point3f(tvecs[i][0], tvecs[i][1], tvecs[i][2]));
                                continue;
                            }
                            else if (ids[i] == 9)
                            {
                                id_to8_t[0] = -0.0345*2;
                                id_to8_t[1] = 0.;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 10)
                            {
                                id_to8_t[0] = 0.0345;
                                id_to8_t[1] = 0.0345;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 11)
                            {
                                id_to8_t[0] = -0.0345;
                                id_to8_t[1] = 0.0345;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 12)
                            {
                                id_to8_t[0] = 0.0345*2;
                                id_to8_t[1] = 0.0345*2;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 13)
                            {
                                id_to8_t[0] = 0.;
                                id_to8_t[1] = 0.0345*2;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 14)
                            {
                                id_to8_t[0] = -0.0345*2;
                                id_to8_t[1] = 0.0345*2;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 15)
                            {
                                id_to8_t[0] = 0.0345;
                                id_to8_t[1] = 0.0345*3;
                                id_to8_t[2] = 0.;
                            }
                            else if (ids[i] == 16)
                            {
                                id_to8_t[0] = -0.0345;
                                id_to8_t[1] = 0.0345*3;
                                id_to8_t[2] = 0.;
                            }
                            cv::Mat id_to8_t_mat{id_to8_t};
                            id_to8_t_mat.convertTo(id_to8_t_mat, CV_32FC1);

                            rotation_matrix.convertTo(rotation_matrix, CV_32FC1);
                            //cv::invert(rotation_matrix, rotation_matrix);
                            cv::Mat id_8_t = rotation_matrix * id_to8_t_mat + vec_t_mat;

                            collected_pt3f.push_back(cv::Point3f(id_8_t.at<float>(0), id_8_t.at<float>(1), id_8_t.at<float>(2)));

                            static tf::TransformBroadcaster br;
                            tf::Transform world2camera = tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), 
                                tf::Vector3(id_8_t.at<float>(0), id_8_t.at<float>(1), id_8_t.at<float>(2)));
                            char obj_str[16];
                            sprintf(obj_str, "object-%d-8", ids[i]);
                            tf::StampedTransform trans_world2camera = tf::StampedTransform(world2camera, ros::Time(pose.header.stamp), "camera", obj_str);
                            br.sendTransform(trans_world2camera);
                        }
                    }
                }
            }


            sensor_msgs::ImagePtr det_output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameCopy).toImageMsg();
            aruco_pub.publish(det_output_msg);
            // cv::imshow("frame", frame);
            // cv::waitKey(10);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
