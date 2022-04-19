//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include <gazebo_msgs/ModelState.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_model_state_example");
    ros::NodeHandle nh("~");

    ros::Publisher model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

    gazebo_msgs::ModelState model_state;
    model_state.model_name = "p450";

    float time = 0;
    while (ros::ok())
    {
        float linear_vel = 0.1;
        float circle_radius = 1.0;
        float omega = fabs(linear_vel / circle_radius);

        const float angle = time * omega;
        const float cos_angle = cos(angle);
        const float sin_angle = sin(angle);

        model_state.pose.position.x = circle_radius * cos_angle + 0;
        model_state.pose.position.y = circle_radius * sin_angle + 0;
        model_state.pose.position.z = 0.05;

        time = time + 0.05;
        ros::Duration(0.05).sleep();
        model_state_pub.publish(model_state);
        ros::spinOnce();
    }

    return 0;

}