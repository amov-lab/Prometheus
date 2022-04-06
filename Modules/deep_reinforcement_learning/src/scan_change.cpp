#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

ros::Publisher output_pub;
sensor_msgs::LaserScan laser_input;
sensor_msgs::LaserScan laser_output;

//回调函数
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{  
    laser_input = *msg;

    laser_output.header = laser_input.header;

    laser_output.angle_min          = -1.8849555921;//-1.570796327;//-1.5707;
    laser_output.angle_max          = 1.8849555921;//1.570796327;//6.28319;
    laser_output.angle_increment    = 0.1570796327;//0.01745330556;   //(3.1414/20=0.15707  ;  3.1414 / 24 = 0.1309  ;  360: 0.2618)
    laser_output.time_increment     = laser_input.time_increment; 
    laser_output.scan_time          = laser_input.scan_time;
    laser_output.range_min          = laser_input.range_min;
    laser_output.range_max          = laser_input.range_max; 
    laser_output.ranges.resize(24);
    laser_output.intensities.resize(24);

    // laser_output.ranges[0] = 1;
    // laser_output.ranges[1] = 1;
    // laser_output.ranges[2] = 1;
    // laser_output.ranges[3] = 1;
    // laser_output.ranges[4] = 1;
    // laser_output.ranges[6] = 1;
    // laser_output.ranges[7] = 1;
    // laser_output.ranges[8] = 1;
    // for(int i=9; i < 360; i++)
    // {
    //     laser_output.ranges[i] = laser_output.range_min;
    // }



    for (int i=0; i<sizeof(laser_input.ranges); i++)
        if(isinf(laser_input.ranges[i]) == 1)
            {laser_input.ranges[i] = laser_input.range_max;}


    int j = 0;
    for(int i=0; i < 24; i++)
    {
        laser_output.ranges[i] = laser_output.range_max;
        laser_output.intensities[i] = 0.0;
        for(j = 68+9*i; j<68+9*(i+1); j++)
        {
            if(laser_output.ranges[i] > laser_input.ranges[j])
            {
                laser_output.ranges[i] = laser_input.ranges[j];
            }
        }
    }



    // for(int i=0; i < 12; i++)
    // {
    //     laser_output.ranges[i] = laser_output.range_max;
    //     laser_output.intensities[i] = 0.0;
    //     for(j = (252+9*i); j<(252+9*(i+1)); j++)
    //     {
    //         if(laser_output.ranges[i] > laser_input.ranges[j])
    //         {
    //             laser_output.ranges[i] = laser_input.ranges[j];
    //         }
    //     }
    // }



    output_pub.publish(laser_output);

}

int main(int argc, char **argv) 
{
    ros::init(argc, argv,"scan_change");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(5.0);
//    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/laserscan_filtered", 1, scanCallback);
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/uav1/scan", 1, scanCallback);
    output_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_output", 10);
            
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }       

  return 0;
} 

