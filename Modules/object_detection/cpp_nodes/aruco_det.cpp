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
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <image_transport/image_transport.h>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>  
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <prometheus_msgs/ArucoInfo.h>
#include <prometheus_msgs/MultiArucoInfo.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "message_utils.h"


using namespace std;
using namespace cv;

//【订阅】输入图像
image_transport::Subscriber image_subscriber;
//【发布】检测得到的位置与姿态信息
ros::Publisher pose_pub;
ros::Publisher multi_pose_pub;
//【发布】输入检测结果图像
image_transport::Publisher aruco_pub;

//【订阅】输入开关量
ros::Subscriber switch_subscriber;
std::string switch_node_name = "/prometheus/object_detection/aruco_navigation_switch";
// 接收消息，允许暂停检测
bool is_suspanded = false;
// 使用cout打印消息
bool local_print = false;
// 使用prometheus_msgs::Message打印消息
bool message_print = true;
//【发布】调试消息
ros::Publisher message_pub;    
std::string msg_node_name = "/prometheus/message/aruco_det";


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
// 保存上一次的检测结果
static float static_aruco_num = 0;
static float static_depth = 0;
static float static_real_x = 0;
static float static_real_y = 0;
static float static_qx = 0;
static float static_qy = 0;
static float static_qz = 0;
static float static_qw = 0;


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

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

// 统一消息输出函数
enum class MSG_TYPE {INFO, WARN, ERROR};
void messageEcho(std::string msg, MSG_TYPE _msg_type)
{
    if (MSG_TYPE::INFO == _msg_type)
    {
        if (local_print)
            ROS_INFO(msg.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, msg);
    }
    else if (MSG_TYPE::WARN == _msg_type)
    {
        if (local_print)
            ROS_WARN(msg.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, msg);
    }
    else if (MSG_TYPE::ERROR == _msg_type)
    {
        if (local_print)
            ROS_ERROR(msg.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::ERROR, msg_node_name, msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_det");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // 主循环的更新频率为60HZ
    ros::Rate loop_rate(60);

    // 发布调试消息
    message_pub = nh.advertise<prometheus_msgs::Message>(msg_node_name, 10);

    // 从launch文件中读取相机参数、输入输出话题名称、aruco参数等
    std::string camera_topic = "/prometheus/camera/rgb/image_raw";
    std::string camera_params_yaml;
    std::string output_pose_topic = "/prometheus/object_detection/aruco_det";
    std::string output_multi_pose_topic = "/prometheus/object_detection/multi_aruco_det";
    std::string output_image_topic = "/prometheus/camera/rgb/image_aruco_det";
    int dictionary_id(2);
    float target_marker_length(0.0207);
    int binding_id(1);

    // 本节点读取的相机话题
    if (nh.getParam("camera_topic", camera_topic))
        messageEcho("camera_topic is: " + camera_topic, MSG_TYPE::INFO);
    else
        messageEcho("didn't find parameter camera_topic", MSG_TYPE::WARN);

    // 相机参数
    if (nh.getParam("camera_parameters", camera_params_yaml))
        messageEcho("camera_parameters is: " + camera_params_yaml, MSG_TYPE::INFO);
    else
        messageEcho("didn't find parameter camera_parameters", MSG_TYPE::WARN);

    // 发布检测结果[位姿]的话题名称-发布单个二维码
    if (nh.getParam("output_pose_topic", output_pose_topic))
        messageEcho("output_pose_topic is: " + output_pose_topic, MSG_TYPE::INFO);
    else
        messageEcho("didn't find parameter output_pose_topic", MSG_TYPE::WARN);

    // 发布检测结果[位姿]的话题名称-同时发布多个二维码
    if (nh.getParam("output_multi_pose_topic", output_multi_pose_topic))
        messageEcho("output_multi_pose_topic is: " + output_multi_pose_topic, MSG_TYPE::INFO);
    else
        messageEcho("didn't find parameter output_multi_pose_topic", MSG_TYPE::WARN);
    
    // 发布检测结果[检测结果图像]的话题名称
    if (nh.getParam("output_image_topic", output_image_topic))
        messageEcho("output_image_topic is: " + output_image_topic, MSG_TYPE::INFO);
    else
        messageEcho("didn't find parameter output_image_topic", MSG_TYPE::WARN);

    char temp_msg[255];
    // 二维码字典
    if (nh.getParam("dictionary_type", dictionary_id)) {
        sprintf(temp_msg, "dictionary_type is: %d", dictionary_id);
        messageEcho(temp_msg, MSG_TYPE::INFO);
    } else {
        messageEcho("didn't find parameter dictionary_type", MSG_TYPE::WARN);
    }

    // 二维码边长大小
    if (nh.getParam("target_marker_length", target_marker_length)) {
        sprintf(temp_msg, "target_marker_length is: %.3f", target_marker_length);
        messageEcho(temp_msg, MSG_TYPE::INFO);
    } else {
        messageEcho("didn't find parameter target_marker_length", MSG_TYPE::WARN);
    }

    // 绑定一个固定的二维码-检测单个二维码
    if (nh.getParam("binding_id", binding_id)) {
        sprintf(temp_msg, "binding_id is: %d", binding_id);
        messageEcho(temp_msg, MSG_TYPE::INFO);
    } else {
        messageEcho("didn't find parameter binding_id", MSG_TYPE::WARN);
    }


    //【订阅】运行状态转换开关
    switch_subscriber = nh.subscribe(switch_node_name.c_str(), 1, switchCallback);
    //【订阅】输入图像
    image_subscriber = it.subscribe(camera_topic.c_str(), 1, cameraCallback);
    //【发布】检测得到的位置与姿态信息
    pose_pub = nh.advertise<prometheus_msgs::ArucoInfo>(output_pose_topic.c_str(), 1);
    multi_pose_pub = nh.advertise<prometheus_msgs::MultiArucoInfo>(output_multi_pose_topic.c_str(), 1);
    //【发布】检测结果图像
    aruco_pub = it.advertise(output_image_topic.c_str(), 1);


    // 初始化开关量
    bool switch_state = is_suspanded;
    // 准备加载相机参数
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    bool readOk = readCameraParameters(camera_params_yaml.c_str(), camera_matrix, distortion_coefficients);
    if (!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
    }
    // 打印已加载的相机参数
    if (local_print) {
        cout << "[camera_matrix]:" << endl;
        cout << camera_matrix << endl;
        cout << "[distortion_coefficients]:" << endl;
        cout << distortion_coefficients << endl;
    }


    // ArUco Marker字典选择
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

    const auto wait_duration = std::chrono::milliseconds(2000);
    // 待发布的检测结果图像
    sensor_msgs::ImagePtr results_immsg;
    // 数据帧
    cv::Mat frame;

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
            messageEcho("Waiting for image.", MSG_TYPE::INFO);
            std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
        }

        if (switch_state != is_suspanded)
        {
            switch_state = is_suspanded;
            if (!is_suspanded)
                messageEcho("Start Detection.", MSG_TYPE::INFO);
            else
                messageEcho("Stop Detection.", MSG_TYPE::INFO);
        }

        if (!is_suspanded)  // 非挂起状态下才执行主函数
        {

        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            frame = cam_image_copy.clone();
        }

        if (!frame.empty())
        {
            std::vector<int> markerids;
            vector<vector<Point2f> > markerCorners, rejectedCandidate;
            Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerids, parameters, rejectedCandidate);

            bool deted = false;
            if (markerids.size()>0)
            {
                vector< Vec3d > rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(markerCorners, target_marker_length, camera_matrix, distortion_coefficients, rvecs, tvecs);

                prometheus_msgs::MultiArucoInfo multi_pose_now;
                multi_pose_now.num_arucos = markerids.size();
                for(unsigned int i = 0; i < markerids.size(); i++)
                {
                    prometheus_msgs::ArucoInfo pose_now;
                    pose_now.header.stamp = ros::Time::now();
                    pose_now.detected = true;
                    pose_now.aruco_num = markerids[i];
                    pose_now.position[0] = tvecs[i][0];
                    pose_now.position[1] = tvecs[i][1];
                    pose_now.position[2] = tvecs[i][2];

                    // 旋转矩阵转四元数
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvecs[i], rotation_matrix);
                    Eigen::Matrix3d rotation_matrix_eigen;
                    cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
                    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix_eigen);
                    q.normalize();

                    pose_now.orientation[0] = q.x();
                    pose_now.orientation[1] = q.y();
                    pose_now.orientation[2] = q.z();
                    pose_now.orientation[3] = q.w();

                    pose_now.sight_angle[0] = atan(tvecs[i][0] / tvecs[i][2]);
                    pose_now.sight_angle[1] = atan(tvecs[i][1] / tvecs[i][2]);

                    if (binding_id == pose_now.aruco_num)
                    {
                        static_aruco_num = markerids[i];
                        static_real_x = tvecs[i][0];
                        static_real_y = tvecs[i][1];
                        static_depth  = tvecs[i][2];
                        static_qx = q.x();
                        static_qy = q.y();
                        static_qz = q.z();
                        static_qw = q.w();
                        pose_pub.publish(pose_now);

                        // 发布TF，可以在Rviz里查看结果
                        static tf::TransformBroadcaster br;
                        tf::Transform world2camera = tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]));
                        tf::StampedTransform trans_world2camera = tf::StampedTransform(world2camera, ros::Time(), "camera", "aruco");
                        br.sendTransform(trans_world2camera);

                        deted = true;
                    }

                    if (local_print) {
                        cout << "flag_detected: " << int(pose_now.detected) << endl;
                        cout << "pos_target: [X Y Z] : " << " " << pose_now.position[0] << " [m] "<< pose_now.position[1] <<" [m] "<< pose_now.position[2] <<" [m] "<< endl;
                    }

                    multi_pose_now.aruco_infos.push_back(pose_now);
                }
                multi_pose_pub.publish(multi_pose_now);
            }
            if (!deted)
            {
                prometheus_msgs::ArucoInfo pose_now;
                pose_now.header.stamp = ros::Time::now();
                pose_now.detected = false;
                pose_now.aruco_num = static_aruco_num;
                pose_now.position[0] = static_real_x;
                pose_now.position[1] = static_real_y;
                pose_now.position[2] = static_depth;

                pose_now.orientation[0] = static_qx;
                pose_now.orientation[1] = static_qy;
                pose_now.orientation[2] = static_qz;
                pose_now.orientation[3] = static_qw;

                pose_now.sight_angle[0] = atan(static_qx / static_depth);
                pose_now.sight_angle[1] = atan(static_qy / static_depth);

                pose_pub.publish(pose_now);

                if (local_print) {
                    cout << "flag_detected: " << int(pose_now.detected) << endl;
                    cout << "pos_target: [X Y Z] : " << " " << pose_now.position[0] << " [m] "<< pose_now.position[1] <<" [m] "<< pose_now.position[2] <<" [m] "<< endl;
                }
            }

            Mat img_copy;
            frame.copyTo(img_copy);
            cv::aruco::drawDetectedMarkers(img_copy, markerCorners, markerids);
            results_immsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_copy).toImageMsg();
            aruco_pub.publish(results_immsg);
            cv::waitKey(1);

		}
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

	// cap.release();

}




