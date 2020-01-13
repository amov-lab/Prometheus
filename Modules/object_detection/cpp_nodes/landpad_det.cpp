/*
降落目标识别程序 需要注意 降落板图片的左侧为冲前
降落板的尺寸为60cmX60cm
Update Time: 2018.10.15
*/
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/calib3d.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "time.h"
#include "fstream"
#include "iostream"
#include "math.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <prometheus_msgs/DetectionInfo.h>
using namespace std;
using namespace cv;

double threshold_error=0.4;

//相机内部参数
float fx,fy,x_0,y_0;
//相机畸变系数
float k1,k2,p1,p2,k3;
//---------------------------variables---------------------------------------
//------------ROS TOPIC---------
//【订阅】无人机位置
ros::Subscriber drone_pose_sub;
//【订阅】小车位置
ros::Subscriber vehicle_pose_sub;
//【订阅】输入图像
image_transport::Subscriber image_subscriber;
//【发布】无人机和小车相对位置
ros::Publisher position_pub;
//【发布】无人机和小车相对偏航角
// ros::Publisher yaw_pub;
//【发布】是否识别到目标标志位
// ros::Publisher position_flag_pub;
//【发布】识别后的图像
image_transport::Publisher landpad_pub;

//-------------VISION-----------
Mat img;



//-------------TIME-------------
ros::Time begin_time;
float cur_time;
float photo_time;


// 是否检测到标志位-----Point.orientation.w=1为检测成功 =0为检测失败
// geometry_msgs::Pose flag_position;
//无人机位姿message
geometry_msgs::Pose pos_drone_optitrack;
Eigen::Vector3d euler_drone_optitrack;
Eigen::Quaterniond q_drone;
//小车位姿message
geometry_msgs::Pose pos_vehicle_optitrack;
Eigen::Vector3d euler_vehicle_optitrack;
Eigen::Quaterniond q_vehicle;

//保存的上次观测的位置 用于cluster算法使用
Eigen::Vector3d last_position;
bool bool_last_position=false;

//-----------------利用Euler角进行三次旋转得到无人机相对目标的位置------------------
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
// 四元数转Euler
// q0 q1 q2 q3
// w x y z
void quaternion_2_euler(Eigen::Quaterniond quat, Eigen::Vector3d &angle)
{
    angle(0) = atan2(2.0 * (quat.z() * quat.y() + quat.w() * quat.x()), 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
    angle(1) = asin(2.0 * (quat.y() * quat.w() - quat.z() * quat.x()));
  //angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    angle(2) = atan2(2.0 * (quat.z() * quat.w() + quat.x() * quat.y()), 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
}

//--------------------------利用optitrack获取真值-------------------------------
//optitrack获取无人机位姿
void optitrack_drone_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pos_drone_optitrack.position.x = msg->pose.position.y;
    pos_drone_optitrack.position.y = msg->pose.position.x;
    pos_drone_optitrack.position.z = - msg->pose.position.z;
    q_drone.w()=msg->pose.orientation.w;
    q_drone.x()=msg->pose.orientation.x;
    q_drone.y()=msg->pose.orientation.y;
    q_drone.z()=msg->pose.orientation.z;
    quaternion_2_euler(q_drone,euler_drone_optitrack);
}
//optitrack获取地面小车位姿
void optitrack_vehicle_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pos_vehicle_optitrack.position.x = msg->pose.position.y;
    pos_vehicle_optitrack.position.y = msg->pose.position.x;
    pos_vehicle_optitrack.position.z = - msg->pose.position.z;
    q_vehicle.w()=msg->pose.orientation.w;
    q_vehicle.x()=msg->pose.orientation.x;
    q_vehicle.y()=msg->pose.orientation.y;
    q_vehicle.z()=msg->pose.orientation.z;
    quaternion_2_euler(q_vehicle,euler_vehicle_optitrack);
}

//获取系统时间
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}


//! Camera related parameters.
int frameWidth_;
int frameHeight_;
std_msgs::Header imageHeader_;
cv::Mat camImageCopy_;
boost::shared_mutex mutexImageCallback_;
bool imageStatus_ = false;
boost::shared_mutex mutexImageStatus_;
// 图像接收回调函数，接收web_cam的话题，并将图像保存在camImageCopy_中
void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("[LandpadDetector] USB image received.");

    cv_bridge::CvImagePtr cam_image;

    try {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        imageHeader_ = msg->header;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cam_image) {
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            camImageCopy_ = cam_image->image.clone();
        }
        {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
            imageStatus_ = true;
        }
        frameWidth_ = cam_image->image.size().width;
        frameHeight_ = cam_image->image.size().height;
    }
    return;
}

// 用此函数查看是否收到图像话题
bool getImageStatus(void)
{
    boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
    return imageStatus_;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "landpad_det");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    drone_pose_sub=nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Quad/pose", 10, optitrack_drone_cb);

    vehicle_pose_sub=nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/vehicle/pose", 30, optitrack_vehicle_cb);

    position_pub=nh.advertise<prometheus_msgs::DetectionInfo>("/vision/target",10);
    // yaw_pub=nh.advertise<geometry_msgs::Pose>("/relative_yaw",10);
    // position_flag_pub=nh.advertise<geometry_msgs::Pose>("/vision/vision_flag",10);

    // 接收图像的话题
    image_subscriber = it.subscribe("/camera/rgb/image_raw", 1, cameraCallback);
    // 发布ArUco检测结果的话题
    landpad_pub = it.advertise("/camera/rgb/image_landpad_det", 1);

    sensor_msgs::ImagePtr msg_ellipse;

    //读取参数文档camera_param.yaml中的参数值；
    nh.param<float>("fx", fx, 582.611780);
    nh.param<float>("fy", fy, 582.283970);
    nh.param<float>("x0", x_0, 355.598968);
    nh.param<float>("y0", y_0, 259.508932);

    nh.param<float>("k1", k1, -0.401900);
    nh.param<float>("k2", k2, 0.175110);
    nh.param<float>("p1", p1, 0.002115);
    nh.param<float>("p2", p2, -0.003032);
    nh.param<float>("k3", k3, 0.0);


    //--------------------------相机参数赋值---------------------
    //相机内参
    Mat camera_matrix;
    camera_matrix =cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0]=582.611780;
    camera_matrix.ptr<double>(0)[2]=355.598968;
    camera_matrix.ptr<double>(1)[1]=582.283970;
    camera_matrix.ptr<double>(1)[2]=259.508932;
    camera_matrix.ptr<double>(2)[2]=1.0f;
    //相机畸变参数k1 k2 p1 p2 k3
    Mat distortion_coefficients;
    distortion_coefficients=cv::Mat(5,1,CV_64FC1,cv::Scalar::all(0));
    distortion_coefficients.ptr<double>(0)[0]=-0.401900;
    distortion_coefficients.ptr<double>(1)[0]=0.175110;
    distortion_coefficients.ptr<double>(2)[0]=0.002115;
    distortion_coefficients.ptr<double>(3)[0]=-0.003032;
    distortion_coefficients.ptr<double>(4)[0]=0.0;


    //ArUco Marker字典选择以及旋转向量和评议向量初始化
    Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(10);
    vector<double> rv(3),tv(3);
    cv::Mat rvec(rv),tvec(tv);
    // cv::VideoCapture capture(0);
    float last_x(0), last_y(0), last_z(0), last_yaw(0);


    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate loopRate(20);
    //----------------------------------------主循环------------------------------------
    const auto wait_duration = std::chrono::milliseconds(2000);
    while (ros::ok())
    {
        while (!getImageStatus()) 
        {
            printf("Waiting for image.\n");
            std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
        }

        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            img = camImageCopy_.clone();
        }

        clock_t start=clock();
        // capture>>img;
        ros::spinOnce();


        //------------------调用ArUco Marker库对图像进行识别--------------
        //markerids存储每个识别到二维码的编号  markerCorners每个二维码对应的四个角点的像素坐标
        std::vector<int> markerids;
        vector<vector<Point2f> > markerCorners,rejectedCandidate;
        Ptr<cv::aruco::DetectorParameters> parameters=cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(img,dictionary,markerCorners,markerids,parameters,rejectedCandidate);

        //-------------------多于一个目标被识别到，进入算法-----------------
        if (markerids.size()>0)
        {
            //未处理后的位置
            vector<cv::Point3f> vec_Position_OcInW;
            vector<double> vec_yaw;
            cv::Point3f A1_Sum_Position_OcInW(0,0,0);
            double A1_Sum_yaw=0.0;
            for(int t=0;t<markerids.size();t++)
            {
                cv::Mat RoteM, TransM;
                //C2W代表 相机坐标系转换到世界坐标系  W2C代表 世界坐标系转换到相机坐标系 Theta为欧拉角
                cv::Point3f Theta_C2W;
                cv::Point3f Theta_W2C;
                cv::Point3f Position_OcInW;


                //--------------对每一个Marker的相对位置进行解算----------------
                vector<vector<Point2f> > singMarkerCorner_10, singMarkerCorner_15;
                if (markerids[t]==1||markerids[t]==3||markerids[t]==7||markerids[t]==9)
                {
                  singMarkerCorner_15.push_back(markerCorners[t]);
                  cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_15,0.15,camera_matrix,distortion_coefficients,rvec,tvec);

                }
                else
                {
                   singMarkerCorner_10.push_back(markerCorners[t]);
                   cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_10,0.10,camera_matrix,distortion_coefficients,rvec,tvec);

                }

                //将解算的位置转化成旋转矩阵 并旋转计算无人机相对于目标的位置
                double rm[9];
                RoteM = cv::Mat(3, 3, CV_64FC1, rm);
                //利用罗德里格斯公式将旋转向量转成旋转矩阵
                Rodrigues(rvec, RoteM);
                double r11 = RoteM.ptr<double>(0)[0];
                double r12 = RoteM.ptr<double>(0)[1];
                double r13 = RoteM.ptr<double>(0)[2];
                double r21 = RoteM.ptr<double>(1)[0];
                double r22 = RoteM.ptr<double>(1)[1];
                double r23 = RoteM.ptr<double>(1)[2];
                double r31 = RoteM.ptr<double>(2)[0];
                double r32 = RoteM.ptr<double>(2)[1];
                double r33 = RoteM.ptr<double>(2)[2];
                TransM = tvec;
                //计算欧拉角
                double thetaz = atan2(r21, r11) / CV_PI * 180;
                double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
                double thetax = atan2(r32, r33) / CV_PI * 180;

                Theta_C2W.z = thetaz;
                Theta_C2W.y = thetay;
                Theta_C2W.x = thetax;

                Theta_W2C.x = -1 * thetax;
                Theta_W2C.y = -1 * thetay;
                Theta_W2C.z = -1 * thetaz;
                //偏移向量
                double tx = tvec.ptr<double>(0)[0];
                double ty = tvec.ptr<double>(0)[1];
                double tz = tvec.ptr<double>(0)[2];
                double x = tx, y = ty, z = tz;

                //进行三次旋转得到相机光心在世界坐标系的位置
                CodeRotateByZ(x, y, -1 * thetaz, x, y);
                CodeRotateByY(x, z, -1 * thetay, x, z);
                CodeRotateByX(y, z, -1 * thetax, y, z);
                Position_OcInW.x = x*-1;
                Position_OcInW.y = y*-1;
                Position_OcInW.z = z*-1;

                //计算偏航角之差
                Eigen::Matrix3d rotateMatrix;
                rotateMatrix<<r11,r12,r13,r21,r22,r23,r31,r32,r33;
                Eigen::Vector3d eulerVec;
                eulerVec(0)=(Theta_C2W.z+90)/180*CV_PI;
                vec_yaw.push_back(eulerVec(0));

                //根据Marker ID对相对位置进行偏移
                switch (markerids[t])
                {
                    case 1:
                    {
                        Position_OcInW.x-=0.175;
                        Position_OcInW.y+=0.175;
                        break;
                    }
                    case 2:
                    {
                        Position_OcInW.y+=0.2;
                        break;
                    }
                    case 3:
                    {
                        Position_OcInW.x+=0.175;
                        Position_OcInW.y+=0.175;
                        break;
                    }
                    case 4:
                    {
                        Position_OcInW.x-=0.2;
                        break;
                    }
                    case 5:
                        break;
                    case 6:
                    {
                        Position_OcInW.x+=0.2;
                        break;
                    }
                    case 7:
                    {
                        Position_OcInW.x-=0.175;
                        Position_OcInW.y-=0.175;
                        break;
                    }
                    case 8:
                    {
                        Position_OcInW.y-=0.2;
                        break;
                    }
                    case 9:
                    {
                        Position_OcInW.x+=0.175;
                        Position_OcInW.y-=0.175;
                        break;
                    }
                }
                //------------switch结束------------
                vec_Position_OcInW.push_back(Position_OcInW);

                A1_Sum_Position_OcInW+=Position_OcInW;
                A1_Sum_yaw+=eulerVec(0); //待修改
                
                
            }
            //解算位置的平均值
            cv::Point3f A1_Position_OcInW(0,0,0);
            double A1_yaw=0.0;
            int marker_count=markerids.size();
            A1_Position_OcInW=A1_Sum_Position_OcInW/marker_count;
            A1_yaw=A1_Sum_yaw/marker_count;


            //将解算后的位置发给控制端
            prometheus_msgs::DetectionInfo pose_now;
            pose_now.detected = true;
            pose_now.frame = 0;
            pose_now.position[0] = -A1_Position_OcInW.x;
            pose_now.position[1] = A1_Position_OcInW.y;
            pose_now.position[2] = A1_Position_OcInW.z;
            pose_now.yaw_error = A1_yaw;

            last_x = pose_now.position[0];
            last_y = pose_now.position[1];
            last_z = pose_now.position[2];
            last_yaw = pose_now.yaw_error;

            // geometry_msgs::Pose point_msg;
            // point_msg.position.x=-A1_Position_OcInW.x;
            // point_msg.position.y=A1_Position_OcInW.y;
            // point_msg.position.z=A1_Position_OcInW.z;
            position_pub.publish(pose_now);

            // geometry_msgs::Pose yaw_msg;
            // yaw_msg.orientation.w=A1_yaw;
            // yaw_pub.publish(yaw_msg);
            cur_time = get_dt(begin_time);
            // flag_position.orientation.w=1;
            // position_flag_pub.publish(flag_position);

            cout<<" flag_detected: "<< int(pose_now.detected) <<endl;
            cout << "pos_target: [X Y Z] : " << " " << pose_now.position[0]  << " [m] "<< pose_now.position[1] <<" [m] "<< pose_now.position[2] <<" [m] "<<endl;

        }
        else
        {
            prometheus_msgs::DetectionInfo pose_now;
            pose_now.detected = false;
            pose_now.frame = 0;
            pose_now.position[0] = last_x;
            pose_now.position[1] = last_y;
            pose_now.position[2] = last_z;
            pose_now.yaw_error = last_yaw;
            // flag_position.orientation.w=0;
            // position_flag_pub.publish(flag_position);
            cout<<" flag_detected: "<< int(pose_now.detected) <<endl;
            cout << "pos_target: [X Y Z] : " << " " << pose_now.position[0]  << " [m] "<< pose_now.position[1] <<" [m] "<< pose_now.position[2] <<" [m] "<<endl;

        }
        //画出识别到的二维码
        cv::aruco::drawDetectedMarkers(img,markerCorners,markerids);
        //计算算法运行时间
        clock_t finish=clock();
        double time=(finish-start)/1000;
        std::cout<<"time="<<time<<std::endl;
        msg_ellipse = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        landpad_pub.publish(msg_ellipse);
        // cv::imshow("test",img);
        cv::waitKey(1);
        loopRate.sleep();
    }
}
