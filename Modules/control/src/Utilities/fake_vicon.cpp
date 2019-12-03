/***************************************************************************************************************************
* fake_vicon.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  test function for fake publish vicon topic
***************************************************************************************************************************/

//头文件
#include <ros/ros.h>
#include <iostream>
#include <math_utils.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;


geometry_msgs::PoseStamped mocap;                              //发送给飞控的mocap(来源：mocap或laser)
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_vicon");
    ros::NodeHandle nh("~");

    // 【订阅】optitrack估计位置
    ros::Publisher optitrack_pub = nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node/UAV/pose", 1000);

    // 频率
    ros::Rate rate(100.0);

    Eigen::Vector3d pos_vicon_enu(0,0,0);

    Eigen::Vector3d Euler_vicon(0,10.0/180*M_PI,10.0/180*M_PI);

    Eigen::Quaterniond q_vicon = quaternion_from_rpy(Euler_vicon);

    // Transform the Quaternion from NED to ENU
    //Eigen::Quaterniond q_vicon_enu = transform_orientation_aircraft_to_baselink( transform_orientation_ned_to_enu(q_vicon) );

    mocap.pose.orientation.x = q_vicon.x();
    mocap.pose.orientation.y = q_vicon.y();
    mocap.pose.orientation.z = q_vicon.z();
    mocap.pose.orientation.w = q_vicon.w();

    mocap.pose.position.x = pos_vicon_enu[0];
    mocap.pose.position.y = pos_vicon_enu[1];
    mocap.pose.position.z = pos_vicon_enu[2];


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>[Fake Vicon]<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

        cout.setf(ios::fixed);

        cout << "Position: " << " " << fixed <<setprecision(5)<< pos_vicon_enu[0] << " [ m ] "<< pos_vicon_enu[1]<<" [ m ] "<<pos_vicon_enu[2]<<" [ m ] "<<endl;
        cout << "Attitude: " << " " << Euler_vicon[2]/M_PI*180<<" [deg] "<<endl;

        optitrack_pub.publish(mocap);
        rate.sleep();
    }

    return 0;

}
