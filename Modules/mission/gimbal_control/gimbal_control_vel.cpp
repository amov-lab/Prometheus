
//ROS 头文件
#include <ros/ros.h>
#include <iostream>

#include "gimbal_control_vel.h"

using namespace std;
using namespace Eigen;

#define NODE_NAME "gimbal_control_vel"
#define PI 3.1415926

Eigen::Matrix<double,6,1> gimbal_att_sp;
Eigen::Vector3d gimbal_att;
Eigen::Vector3d gimbal_att_deg;
Eigen::Vector3d gimbal_att_rate;
Eigen::Vector3d gimbal_att_rate_deg;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control_vel");
    ros::NodeHandle nh("~");

    // 节点运行频率： 10hz 
    ros::Rate rate(10.0);

    gimbal_control_vel gimbal_control_;

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

    float gimbal_att_sp_deg[6];

    // 
    while(ros::ok())
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Gimbal Control Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter gimbal attitude: "<<endl;
        cout << "Roll [deg], Pitch [deg], Yaw [deg]: "<<endl;
        cin >> gimbal_att_sp_deg[0] >> gimbal_att_sp_deg[1] >> gimbal_att_sp_deg[2];
        cout << "Roll_vel [deg], Pitch_vel [deg], Yaw_vel [deg]: "<<endl;
        cin >> gimbal_att_sp_deg[3] >> gimbal_att_sp_deg[4] >> gimbal_att_sp_deg[5];
        
        // 注意必须要解锁飞机后才可控制云台
        // 注意：使用角度值直接进行控制
        // 面向相机朝向的方向，逆时针旋转roll为正，但roll通道一般不控制
        // pitch向上抬头为正
        // 面向相机朝向的方向，向右转头yaw为正(与飞机yaw相反)
        gimbal_att_sp[0] = gimbal_att_sp_deg[0];
        gimbal_att_sp[1] = gimbal_att_sp_deg[1];
        gimbal_att_sp[2] = gimbal_att_sp_deg[2];
        gimbal_att_sp[3] = gimbal_att_sp_deg[3];
        gimbal_att_sp[4] = gimbal_att_sp_deg[4];
        gimbal_att_sp[5] = gimbal_att_sp_deg[5];

        gimbal_control_.send_mount_control_command(gimbal_att_sp);

        cout << "gimbal_att_sp : " << gimbal_att_sp_deg[0] << " [deg] "<< gimbal_att_sp_deg[1] << " [deg] "<< gimbal_att_sp_deg[2] << " [deg] "<<endl;

        for (int i=0; i<10; i++)
        {
            gimbal_att = gimbal_control_.get_gimbal_att();
            gimbal_att_deg = gimbal_att/PI*180;
            cout << "gimbal_att         : " << gimbal_att_deg[0] << " [deg] "<< gimbal_att_deg[1] << " [deg] "<< gimbal_att_deg[2] << " [deg] "<<endl;
            gimbal_att_rate = gimbal_control_.get_gimbal_att_rate();
            gimbal_att_rate_deg = gimbal_att_rate/PI*180;
            cout << "gimbal_att_rate    : " << gimbal_att_rate_deg[0] << " [deg/s] "<< gimbal_att_rate_deg[1] << " [deg/s] "<< gimbal_att_rate_deg[2] << " [deg/s] "<<endl;
            ros::spinOnce();
            rate.sleep();
        }


    }

    return 0;

}
