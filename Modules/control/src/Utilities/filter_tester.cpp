#include <ros/ros.h>

#include <iostream>
#include <Filter/LowPassFilter.h>
#include <prometheus_control_utils.h>
#include <geometry_msgs/Point.h>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_tester");
    ros::NodeHandle nh;

    geometry_msgs::Point random;

    ros::Publisher log_pub = nh.advertise<geometry_msgs::Point>("/prometheus/test", 10);

    ros::Rate rate(50.0);

    float T1 = 0.1;
    float T2 = 1;

    LowPassFilter LPF1;
    LowPassFilter LPF2;
    LowPassFilter LPF3;

    LPF1.set_Time_constant(T2);
    LPF2.set_Time_constant(T2);
    LPF3.set_Time_constant(T2);


    float dt = 1;

    float input1,input2;
    float output1,output2;

    while(ros::ok())
    {

        // cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        // cout << "Input the input of the LPF1"<<endl;
        // cin >> input1;

        // cout << "Input the input of the LPF2"<<endl;
        // cin >> input2;

        // output1 = LPF1.apply(input1,dt);

        // output2 = LPF2.apply(input2,dt);

        // float _T1 = LPF1.get_Time_constant();
        // float _T2 = LPF2.get_Time_constant();


        // cout << "T for LPF1: "<< _T1 <<endl;
        // cout << "T for LPF2: "<< _T2 <<endl;

        // cout << "ouput for LPF1: "<< output1 <<endl;
        // cout << "ouput for LPF2: "<< output2 <<endl;


        // 先生成随机数
        random.x = prometheus_control_utils::random_num(0.2, 0.1);
        random.y = prometheus_control_utils::random_num(2, 0.0);
        random.z = prometheus_control_utils::random_num(0.1, 0.05);

        // 低通滤波
        random.x = LPF1.apply(random.x , 0.02);
        random.y = LPF2.apply(random.y , 0.02);
        random.z = LPF3.apply(random.z , 0.02);

        ROS_INFO("random.x: [%f]", random.x);
        ROS_INFO("random.y: [%f]", random.y);
        ROS_INFO("random.z: [%f]", random.z);


        log_pub.publish(random);



        rate.sleep();
    }

    return 0;
}
