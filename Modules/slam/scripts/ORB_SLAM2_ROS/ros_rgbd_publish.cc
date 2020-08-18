/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "../../../include/Converter.h"

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System *pSLAM, ros::NodeHandle *pNH) : mpSLAM(pSLAM), myNH(pNH)
    {
        pose_pub = pNH->advertise<geometry_msgs::PoseStamped>("slam/pose", 5);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    ORB_SLAM2::System *mpSLAM;
    ros::NodeHandle *myNH;
    ros::Publisher pose_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Publish");
    ros::start();

    if (argc != 3)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);

    ros::NodeHandle nh;

    ImageGrabber igb(&SLAM, &nh);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/realsense_plugin/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/realsense_plugin/camera/depth/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    if (Tcw.empty())
    {
        return;
    }
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map_slam";

    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();

    cv::Mat Rmap = (cv::Mat_<float>(3, 3) << 0, 0, 1.0, -1.0, 0, 0, 0, -1.0, 0);
    // Rwc = Rmap * Rwc;
    cv::Mat twc = -Rwc * (Tcw.rowRange(0, 3).col(3));
    twc = Rmap * twc;
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    tf::Transform new_transform;
    // tf::Transform map_transform(tf::Matrix3x3(0, 0, 1.0, -1.0, 0, 0, 0, -1.0, 0));
    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));
    tf::Quaternion quaternion(q[2], -q[0], -q[1], q[3]);

    new_transform.setRotation(quaternion);
    //new_transform.mult(map_transform, new_transform);
    tf::poseTFToMsg(new_transform, pose.pose);

    pose_pub.publish(pose);
}
