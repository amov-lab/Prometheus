#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "elas.h"
#include "stereomatch.h"
#include "pointcloud.h"

ros::Publisher k_depth_pub;
ros::Publisher pcl_pub;
StereoPointCloud stereoPointCloud;
PointCloud* Ptr = stereoPointCloud.pointCloud;

template <class Type>  
Type stringToNum(const std::string& str)
{  
    std::istringstream iss(str); 
    Type num;  
    iss >> num;  
    return num;      
}  

class ImageProcessor
{
public:
    ImageProcessor(float fx, float baseline): fx(fx), baseline(baseline){}
    void processStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight, const geometry_msgs::PoseStampedConstPtr& msgPose);

private:
    float fx, fy, cx, cy;
    float baseline;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sineva_stereo_node");
    ros::NodeHandle nh;

    std::string focal_length, base_line, leftTopic, rightTopic, poseTopic;
    nh.param<std::string>("sineva_stereo_node/focal_length", focal_length, "711.9");
    nh.param<std::string>("sineva_stereo_node/base_line", base_line, "0.12");
//    nh.param<std::string>("sineva_stereo_node/left_topic", leftTopic, "/cam0/image_raw");
//    nh.param<std::string>("sineva_stereo_node/right_topic", rightTopic, "/cam1/image_raw");
    nh.param<std::string>("sineva_stereo_node/left_topic", leftTopic, "/mynteye/left/image_mono");
    nh.param<std::string>("sineva_stereo_node/right_topic", rightTopic, "/mynteye/right/image_mono");
//   nh.param<std::string>("sineva_stereo_node/pose_topic", poseTopic, "/maplab_rovio/T_M_I");
    nh.param<std::string>("sineva_stereo_node/pose_topic", poseTopic, "/posestamped");
    ImageProcessor imageProcesser(stringToNum<float>(focal_length),stringToNum<float>(base_line));
    //depth publisher
    k_depth_pub = nh.advertise<sensor_msgs::Image>("sineva_depth", 1);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1); 
    //point cloud publisher
    //subscribe to left and right image
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, leftTopic, 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, rightTopic, 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, poseTopic, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, 
    geometry_msgs::PoseStamped> SyncPol;
    message_filters::Synchronizer<SyncPol> sync(SyncPol(10), left_sub, right_sub, pose_sub);
    sync.registerCallback(boost::bind(&ImageProcessor::processStereo, &imageProcesser, _1, _2, _3));
    std::cout << "Storing for point cloud" << std::endl;
    ros::spin();

    return 0;
}



// void publishPointCloud(const cv::Mat &disparityImage, const Eigne::Isometry3d &Trans, )

void ImageProcessor::processStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight, const geometry_msgs::PoseStampedConstPtr& msgPose)
{
    // convert the ros image message to cv::Mat using cvshared
    cv::Mat leftImage, rightImage;
    ofstream f;
    std::string filename = "tum_mynt.txt";
    double x,y,z,qx,qy,qz,qw;
    double timestamp;
    try
    {
        leftImage = cv_bridge::toCvShare(msgLeft)->image;
        rightImage = cv_bridge::toCvShare(msgRight)->image;
        x = msgPose->pose.position.x;
        y = msgPose->pose.position.y;
        z = msgPose->pose.position.z;
        qx = msgPose->pose.orientation.x;
        qy = msgPose->pose.orientation.y;
        qz = msgPose->pose.orientation.z;
        qw = msgPose->pose.orientation.w;
        timestamp = msgPose->header.stamp.toSec();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(leftImage.channels() == 3)
    {
        cv::cvtColor(leftImage, leftImage, CV_BGR2GRAY);
        cv::cvtColor(rightImage, rightImage, CV_BGR2GRAY);
    }
    // Save the data to tum file
    // f.open(filename,ios::app);
    // f << std::fixed;
    // f << setprecision(6) << timestamp << " " << setprecision(9) << x << " "
    // << y << " " << z << " " << qx << " " << qy << " "<< qz << " " << qw << std::endl; 
    // f.close();
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Isometry3d Trans(q);
    Trans.pretranslate( Eigen::Vector3d(x,y,z)); 
    StereoMatch stereoMatch;
    //StereoPointCloud stereoPointCloud;
    cv::Mat depthImage = stereoMatch.run(leftImage, rightImage, baseline, fx);
    sensor_msgs::ImagePtr depthMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthImage).toImageMsg();
    
    //PointCloud* locPC = stereoPointCloud.addNewPoints_local(depthImage);
    stereoPointCloud.addNewPoints_disparity(depthImage, Trans);
    Ptr = stereoPointCloud.pointCloud;
    sensor_msgs::PointCloud2 outputPC;
    pcl::toROSMsg(*Ptr,outputPC);
    outputPC.header.stamp = ros::Time::now();
    outputPC.header.frame_id = "odom";
    k_depth_pub.publish(depthMsg);
    pcl_pub.publish(outputPC);
}
