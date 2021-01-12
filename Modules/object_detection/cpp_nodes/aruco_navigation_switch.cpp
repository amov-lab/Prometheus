/***************************************************************************************************************************
 * aruco_navigation_switch.cpp
 * Author: Jario
 * Update Time: 2021.01.12
 *
 * 说明:
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

// #include "message_utils.h"


using namespace std;
using namespace cv;


// 使用cout打印消息
bool local_print = true;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_navigation_switch");
    ros::NodeHandle nh("~");

    // 更新频率为60HZ
    ros::Rate loop_rate(60);

    //【发布】检测得到的位置与姿态信息
    ros::Publisher switch_pub = nh.advertise<std_msgs::String>("/prometheus/object_detection/aruco_navigation_switch", 1);

    
    while (ros::ok())
	{
        char ch;
        string help_str = "|------------------------------------|\n"
                          "| KEYS:                              |\n"
                          "| 'c' Calibrate world frame          |\n"
                          "| 'q' Quit                           |\n"
                          "|------------------------------------|\n"
                          "Input: ";
        cout << help_str;

        cin >> ch;
        if ('c' == ch)
        {
            std_msgs::String msg;
            msg.data = "calibrate";
            switch_pub.publish(msg);
            cout << "SEND calibrate!" << endl;
        }
        else if ('q' == ch)
        {
            cout << "Bye!" << endl;
            break;
        }
        else
        {

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
