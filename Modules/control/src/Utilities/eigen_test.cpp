
//头文件
#include <ros/ros.h>


#include <iostream>

//话题头文件

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/State.h>

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>


using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);
    Eigen::Vector3d vector;

    vector       = Eigen::Vector3d(0.0,0.0,0.0);


    //float a;

    Eigen::Vector3d thr_sp;
    float yaw_sp;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {

        cout << "Please input the thrust_x: "<<endl;
        cin >> thr_sp(0);
        cout << "Please input the thrust_y: "<<endl;
        cin >> thr_sp(1);
        cout << "Please input the thrust_z: "<<endl;
        cin >> thr_sp(2);
        cout << "Please input the yaw_sp: "<<endl;
        cin >> yaw_sp;


        //Eigen::Quaterniond q_sp = ThrottleToAttitude(thr_sp, yaw_sp);

        //周期休眠
        rate.sleep();
    }

    return 0;

}
