
//ROS 头文件
#include <ros/ros.h>
#include <iostream>

#include "gimbal_control.h"

using namespace std;
using namespace Eigen;

#define NODE_NAME "gimbal_control"
#define PI 3.1415926

Eigen::Vector3d gimbal_att_sp;
Eigen::Vector3d gimbal_att;
Eigen::Vector3d gimbal_att_deg;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control");
    ros::NodeHandle nh("~");

    // 节点运行频率： 10hz 
    ros::Rate rate(10.0);

    gimbal_control gimbal_control_;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    float gimbal_att_sp_deg[3];

    while(ros::ok())
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Gimbal Control Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter gimbal attitude: "<<endl;
        cout << "Pitch [deg] "<<endl;
        cin >> gimbal_att_sp_deg[0];
        cout << "Roll [deg] "<<endl;
        cin >> gimbal_att_sp_deg[1];    
        cout << "Yaw [deg] "<<endl;
        cin >> gimbal_att_sp_deg[2];

        gimbal_att_sp[0] = gimbal_att_sp_deg[0];
        gimbal_att_sp[1] = gimbal_att_sp_deg[1];
        gimbal_att_sp[2] = gimbal_att_sp_deg[2];

        gimbal_control_.send_mount_control_command(gimbal_att_sp);

        


        cout << "gimbal_att_sp : " << gimbal_att_sp_deg[0] << " [deg] "<< gimbal_att_sp_deg[1] << " [deg] "<< gimbal_att_sp_deg[2] << " [deg] "<<endl;

        for (int i=0; i<5; i++)
        {
            gimbal_att = gimbal_control_.get_gimbal_att();
            gimbal_att_deg = gimbal_att/PI*180;
            cout << "gimbal_att    : " << gimbal_att_deg[0] << " [deg] "<< gimbal_att_deg[1] << " [deg] "<< gimbal_att_deg[2] << " [deg] "<<endl;
            ros::spinOnce();
            rate.sleep();
        }


    }

    return 0;

}
