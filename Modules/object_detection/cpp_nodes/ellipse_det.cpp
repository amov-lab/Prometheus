/***************************************************************************************************************************
 * ellipse_det.cpp
 * Author: Jario
 * Update Time: 2020.1.14
 *
 * 说明: 椭圆识别程序(同时可以判断椭圆中心标识，需要训练)，具体训练方式见Prometheus/Modules/object_detection/README.md
 *      1. 【订阅】图像话题 (默认来自web_cam)
 *         /prometheus/camera/rgb/image_raw
 *      2. 【发布】目标位置，发布话题见 Prometheus/Modules/msgs/msg/DetectionInfo.msg
 *         /prometheus/object_detection/ellipse_det
 *      3. 【发布】检测结果的可视化图像话题
 *         /prometheus/camera/rgb/image_ellipse_det
***************************************************************************************************************************/
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
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
#include <prometheus_msgs/DetectionInfo.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml.hpp>
#include "spire_ellipsedetector.h"
#include "message_utils.h"


using namespace std;
using namespace cv;
using namespace cv::ml;
using namespace spire;


// #define MARKER_SIZE 0.18

#define ELLIPSE_DET
#define ELLIPSE_PUB

// circle eg. IM_NAME.jpg 0 is background 1 is circle
// std::string imlist_dir = "/home/nvidia/vision_ws/src/ellipse_det_ros/labeled_img_class.txt";  // 进行一个字符串赋值，方便下面调用
// images, include above information
std::string base_path = "/home/nvidia/vision_ws/src/ellipse_det_ros/images_from_camera/";  // 同上

//【订阅】输入开关量
ros::Subscriber switch_subscriber;
// 接收消息，允许暂停检测
bool is_suspanded = false;
// 使用cout打印消息
bool local_print = true;
// 使用prometheus_msgs::Message打印消息
bool message_print = true;
//【发布】调试消息
ros::Publisher message_pub;
std::string msg_node_name;

// 相机话题中的图像同步相关变量
int frame_width, frame_height;
std_msgs::Header image_header;
cv::Mat cam_image_copy;
boost::shared_mutex mutex_image_callback;
bool image_status = false;
boost::shared_mutex mutex_image_status;

EllipseDetector ellipse_detector;
HOGDescriptor hog(Size(28, 28), Size(4, 4), Size(4, 4), Size(4, 4), 9);
bool use_hog = true;


// 图像接收回调函数，接收web_cam的话题，并将图像保存在cam_image_copy中
void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (local_print)
        ROS_DEBUG("[EllipseDetector] USB image received.");

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

void ellipse_det(cv::Mat& input, cv::Mat& output, std::vector<Ellipse>& ells)
{
    static bool first_im = true;
    if (first_im)
    {
        float   fThScoreScore = 0.7f;   // 0.8
        float   fMinReliability = 0.4f; // Const parameters to discard bad ellipses 0.4
        float   fTaoCenters = 0.04f;    // 0.05     
        int     iThLength = 16;         // 16
        float   fMinOrientedRectSide = 3.0f;
        int     iNs = 16;
        Size    szPreProcessingGaussKernelSize = Size(5, 5);
        double  dPreProcessingGaussSigma = 1.0;
        float   fThPos = 1.0f;
        float   fDistanceToEllipseContour = .1f;

        Size sz = input.size();
        
        float fMaxCenterDistance = sqrt(float(sz.width*sz.width + sz.height*sz.height)) * fTaoCenters;
        // Initialize Detector with selected parameters
        ellipse_detector.SetParameters(szPreProcessingGaussKernelSize,
            dPreProcessingGaussSigma,
            fThPos,
            fMaxCenterDistance,
            iThLength,
            fMinOrientedRectSide,
            fDistanceToEllipseContour,
            fThScoreScore,
            fMinReliability,
            iNs
        );
        first_im = false;
    }
    
    input.copyTo(output);
    ellipse_detector.Detect(input, ells);
    ellipse_detector.DrawDetectedEllipses(output, ells);
}

vector<float> hist_feature(cv::Mat& resized_im)  // 生成直方图，带入形参 Mat resized_im
{
    float feats0[10], feats1[10], feats2[10];    // 创建三个数组，用于承载三个通道的数据
    for (int i = 0; i < 10; i++)  // 全部赋值为0
    {
        feats0[i] = 0; feats1[i] = 0; feats2[i] = 0;
    }
    for (int i = 0; i < resized_im.rows; i++) for (int j = 0; j < resized_im.cols; j++)  // 对形参Mat resized_im图像的每一个像素点都进行遍历
    {
        uchar s = resized_im.ptr<uchar>(i, j)[0];  // 并将每一个像素的通道中的值赋值给SHV
        uchar h = resized_im.ptr<uchar>(i, j)[1];
        uchar v = resized_im.ptr<uchar>(i, j)[2];

        float gird = (256. / 10.);         // 设置一个参数，用于分割各通道。这里之所以除以10,是因为feat0,feat1,feat2数组长度设置为10。
        int ind = (int)((float)s / gird);  // 将feat数组中对应的数值进行++
        feats0[ind] ++;

        ind = (int)((float)h / gird);
        feats1[ind] ++;

        ind = (int)((float)v / gird);
        feats2[ind] ++;
    }

    vector<float> feats(30, 0); float total(0);  // 创建一个vector，长度为30,全部为0;创建一个变量total=0
    for (int i = 0; i < 30; i++)                 // 就是将feat0,feat0，feat2中30个数的值依次赋值给feats，并全部相加赋值给total，为了下一步的归一化做准备。
    {
        if (i < 10)
        {
            feats[i] = feats0[i];
            total += feats[i];
        }
        else if (i < 20)
        {
            feats[i] = feats1[i-10];
            total += feats[i];
        }
        else
        {
            feats[i] = feats2[i-20];
            total += feats[i];
        }
    }

    for (int i = 0; i < 30; i++)  // 进行归一化
    {
        feats[i] /= total;
    }

    if (use_hog)
    {
        cv::Mat resized_im_gray;
        cvtColor(resized_im, resized_im_gray, CV_BGR2GRAY);

        // cout << "w, h: " << resized_im_gray.cols << ", " << resized_im_gray.rows << endl;
        vector<float> descriptors;  // HOG描述子向量
        hog.compute(resized_im_gray, descriptors, Size(4, 4));
        feats.insert(
            feats.end(),
            std::make_move_iterator(descriptors.begin()),
            std::make_move_iterator(descriptors.end())
        );
        // cout << "descriptors size" << descriptors.size() << endl;
        // cout << "feats size" << feats.size() << endl;
    }
    return feats;  // 返回feats
}

// 用标注的图像训练SVM分类器
Ptr<SVM> train_svm_classifier(std::string train_imlist, std::string train_imdir)
{
    ifstream ifs(train_imlist);
    string im_name;
    int label;

    Mat resized_im;
    
    vector<vector<float> > all_feats;
    vector<int> all_labels;

    while (ifs >> im_name >> label)
    {
        cout << im_name << " " << label << endl;
        cout << train_imdir + "/" + im_name << endl;
        Mat im_one = imread(train_imdir + "/" + im_name, 1);
        all_labels.push_back(label);

        cv::resize(im_one, resized_im, Size(28, 28));
        cv::cvtColor(resized_im, resized_im, COLOR_BGR2HSV);

        vector<float> feats = hist_feature(resized_im);

        all_feats.push_back(feats);
    }

    int feats_len = 30;
    if (use_hog)
    {
        feats_len = 471;
    }
    Mat trainingDataMat(all_feats.size(), feats_len, CV_32FC1);
    Mat labelsMat(all_feats.size(), 1, CV_32S);
    for (int i = 0; i < all_feats.size(); i++) {
        for (int t = 0; t < feats_len; t++) {
            float tmp = all_feats[i][t]; // !!!!!!!!!!!!!!!
            float* pf = trainingDataMat.ptr<float>(i, t);
            *pf = tmp;
        }

        int gt = all_labels[i]; // !!!!!!!!!!!!!!!
        int* pi = labelsMat.ptr<int>(i);
        *pi = gt;
    }

    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
    //! [init]
    //! [train]
    svm->train(trainingDataMat, ROW_SAMPLE, labelsMat);
    // svm->save("trained_svm.xml");

    return svm;
}

void switchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    is_suspanded = !(bool)msg->data;
    // cout << is_suspanded << endl;
}

//! ROS subscriber and publisher.
image_transport::Subscriber imageSubscriber_;
#ifdef ELLIPSE_PUB
image_transport::Publisher ellipse_pub;
#endif
ros::Publisher pose_pub;
// cv::ml::SVM
Ptr<SVM> svm;

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "ellipse_det");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // 发布调试消息
    msg_node_name = "/prometheus/message/ellipse_det";
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

    // With Training
    bool wt = false;
    if (nh.getParam("with_training", wt)) {
        if (local_print)
            ROS_INFO("with_training is %d", wt);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter with_training");
        wt = false;
    }

    std::string train_imlist, train_imdir;
    if (nh.getParam("train_imlist", train_imlist)) {
        if (local_print)
            ROS_INFO("train_imlist is %s", train_imlist.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "train_imlist is" + train_imlist);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter train_imlist");
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, "didn't find parameter train_imlist");
        train_imlist = "ellipse/labeled_img_class.txt";
    }

    if (nh.getParam("train_imdir", train_imdir)) {
        if (local_print)
            ROS_INFO("train_imdir is %s", train_imdir.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "train_imdir is" + train_imdir);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter train_imdir");
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, "didn't find parameter train_imdir");
        train_imdir = "ellipse/images_from_camera/";
    }

    // Saving ellipse center
    bool saving_center = false;
    if (nh.getParam("saving_center", saving_center)) {
        if (local_print)
            ROS_INFO("saving_center is %d", saving_center);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter saving_center");
        saving_center = false;
    }

    std::string saving_path;
    if (nh.getParam("saving_path", saving_path)) {
        if (local_print)
            ROS_INFO("saving_path is %s", saving_path.c_str());
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "saving_path is" + saving_path);
    } else {
        if (local_print)
            ROS_WARN("didn't find parameter saving_path");
        if (message_print)
            pub_message(message_pub, prometheus_msgs::Message::WARN, msg_node_name, "didn't find parameter saving_path");
        saving_path = "ellipse/images_from_camera/";
    }


    bool switch_state = is_suspanded;
    // 接收开关话题
    switch_subscriber = nh.subscribe("/prometheus/switch/ellipse_det", 10, switchCallback);

    // for (int i=0; i<argc; i++)
    // {
    //     if (strcmp("wt", argv[i]) == 0) { wt = true; break; }
    // }


    if (wt)
        svm = train_svm_classifier(train_imlist, train_imdir);

    ros::Rate loop_rate(30);
    
    std::string ros_path = ros::package::getPath("prometheus_detection");
    if (local_print)
        cout << "DETECTION_PATH: " << ros_path << endl;
    if (message_print)
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_node_name, "DETECTION_PATH: " + ros_path);

    //读取参数文档camera_param.yaml中的参数值；
    YAML::Node camera_config = YAML::LoadFile(camera_info);
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

    double ellipse_det_r = camera_config["ellipse_det_r"].as<double>();


    // 接收图像的话题
    imageSubscriber_ = it.subscribe(camera_topic, 1, cameraCallback);
#ifdef ELLIPSE_PUB
    // 发布椭圆检测结果的话题
    ellipse_pub = it.advertise("/prometheus/camera/rgb/image_ellipse_det", 1);
#endif
    // 椭圆检测结果，xyz
    pose_pub = nh.advertise<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/ellipse_det", 1);
    
    sensor_msgs::ImagePtr msg_ellipse;

    int saving_ellipse_cnt = 0;
    // const auto wait_duration = std::chrono::milliseconds(2000);
    ros::Rate loopRate_1Hz(1);
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

#ifdef ELLIPSE_DET
        Mat ellipse_show, frame;
        std::vector<Ellipse> ells, ells_copy;
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            frame = cam_image_copy.clone();
        }
        ellipse_det(frame, ellipse_show, ells);

        static double last_x(0), last_y(0), last_z(0);
        bool deted = false;

        if (saving_center)
        {
            // 遍历每个检测到的椭圆
            for (int i=0; i<ells.size(); i++)
            {
                Ellipse e = ells[i];

                int xc = ells[i].xc_;
                int yc = ells[i].yc_;
                int width = (ells[i].a_ + ells[i].b_)*0.65;

                // 提取椭圆中心区域
                Rect output_r(xc - width / 2, yc - width / 2, width, width);
                if (output_r.x < 0 || output_r.y < 0 ||
                    output_r.x + output_r.width >= frame.cols ||
                    output_r.y + output_r.height >= frame.rows)
                    continue;
                Mat center_det = frame(output_r);

                // 提取中心区域的颜色直方图
                cv::resize(center_det, center_det, cv::Size(28,28));

                char output_name[256];
                sprintf(output_name, "%s/%010d.jpg", saving_path.c_str(), saving_ellipse_cnt++);
                cv::imwrite(output_name, center_det);
            }
        }
        else if (wt)
        {
            // 遍历每个检测到的椭圆
            for (int i=0; i<ells.size(); i++)
            {
                Ellipse e = ells[i];

                int xc = ells[i].xc_;
                int yc = ells[i].yc_;
                int width = (ells[i].a_ + ells[i].b_)*0.65;

                // 提取椭圆中心区域
                Rect output_r(xc - width / 2, yc - width / 2, width, width);
                if (output_r.x < 0 || output_r.y < 0 ||
                    output_r.x + output_r.width >= frame.cols ||
                    output_r.y + output_r.height >= frame.rows)
                    continue;
                Mat center_det = frame(output_r);

                // 提取中心区域的颜色直方图
                cv::resize(center_det, center_det, cv::Size(28,28));
                cv::cvtColor(center_det, center_det, COLOR_BGR2HSV);
                std::vector<float> feat = hist_feature(center_det);

                int feats_len = 30;
                if (use_hog)
                {
                    feats_len = 471;
                }
                cv::Mat predictDataMat(1, feats_len, CV_32F);
                for (int i=0; i<feats_len; i++)
                {
                    predictDataMat.at<float>(i) = feat[i];
                }
                // 利用SVM分类器判断是否是需要的检测结果
                float pred = svm->predict(predictDataMat);

                // 如果是的，则将结果发布出去
                if (pred == 1)
                {
                    deted = true;
                    
                    float theta_x = atan((e.xc_ - cx) / fx);  //315.06 calibration
                    float theta_y = atan((e.yc_ - cy) / fy);  //241.27 calibration 

                    float depth = ellipse_det_r*fx/e.b_; // shendu

                    float real_x = depth*tan(theta_x);
                    float real_y = depth*tan(theta_y);

                    // geometry_msgs::Pose pose_now;
                    prometheus_msgs::DetectionInfo pose_now;
                    pose_now.header.stamp = ros::Time::now();
                    // pose_now.orientation.w = 1;
                    // pose_now.position.x = depth;
                    // pose_now.position.y = real_x;
                    // pose_now.position.z = real_y;
                    pose_now.detected = true;
                    pose_now.frame = 0;
                    pose_now.position[0] = real_x;
                    pose_now.position[1] = real_y;
                    pose_now.position[2] = depth;
                    pose_pub.publish(pose_now);

                    last_x = real_x;
                    last_y = real_y;
                    last_z = depth;

                    //cout << "flag_detected: " << int(pose_now.detected) <<endl;
                    //cout << "pos_target: [X Y Z] : " << " " << pose_now.position[0] << " [m] "<< pose_now.position[1] <<" [m] "<< pose_now.position[2] <<" [m] "<<endl;

                    ells_copy.push_back(e);
                }
            }            
        }
        else if (ells.size() > 0)
        {
            Ellipse e = ells[0];
            deted = true;
            
            float theta_x = atan((e.xc_ - cx) / fx);  //315.06 calibration
            float theta_y = atan((e.yc_ - cy) / fy);  //241.27 calibration 

            float depth = ellipse_det_r*fx/e.b_; // shendu

            float real_x = depth*tan(theta_x);
            float real_y = depth*tan(theta_y);

            // geometry_msgs::Pose pose_now;
            prometheus_msgs::DetectionInfo pose_now;
            pose_now.header.stamp = ros::Time::now();
            // pose_now.orientation.w = 1;
            // pose_now.position.x = depth;
            // pose_now.position.y = real_x;
            // pose_now.position.z = real_y;
            pose_now.detected = true;
            pose_now.frame = 0;
            pose_now.position[0] = real_x;
            pose_now.position[1] = real_y;
            pose_now.position[2] = depth;
            pose_pub.publish(pose_now);

            last_x = real_x;
            last_y = real_y;
            last_z = depth;

            //cout << "flag_detected: " << int(pose_now.detected) <<endl;
            //cout << "pos_target: [X Y Z] : " << " " << pose_now.position[0] << " [m] "<< pose_now.position[1] <<" [m] "<< pose_now.position[2] <<" [m] "<<endl;

            ells_copy.push_back(e);
        }
        if (!deted)
        {
            // 如果没检测到，则发布上次的检测结果，并用标志orientation.w = 0告知未检测到
            // geometry_msgs::Pose pose_now;
            prometheus_msgs::DetectionInfo pose_now;
            pose_now.header.stamp = ros::Time::now();
            // pose_now.orientation.w = 0;
            // pose_now.position.x = last_x;
            // pose_now.position.y = last_y;
            // pose_now.position.z = last_z;
            pose_now.detected = false;
            pose_now.frame = 0;
            pose_now.position[0] = last_x;
            pose_now.position[1] = last_y;
            pose_now.position[2] = last_z;
            pose_pub.publish(pose_now);

            //cout << "flag_detected: " << int(pose_now.detected) <<endl;
            //cout << "pos_target: [X Y Z] : " << " " << pose_now.position[0] << " [m] "<< pose_now.position[1] <<" [m] "<< pose_now.position[2] <<" [m] "<<endl;

        }
#ifdef ELLIPSE_PUB
        // 发布椭圆检测的可视化结果，用于测试
        Mat img_copy;
        frame.copyTo(img_copy);
        ellipse_detector.DrawDetectedEllipses(img_copy, ells_copy);
        msg_ellipse = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_copy).toImageMsg();  
        ellipse_pub.publish(msg_ellipse); 
#endif
#endif
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}




