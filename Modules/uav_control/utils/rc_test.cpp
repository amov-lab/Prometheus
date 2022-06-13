#include <ros/ros.h>

#include "rc_input.h"
#include <mavros_msgs/RCIn.h>

RC_Input rc_input;

void px4_rc_cb(const mavros_msgs::RCIn::ConstPtr &msg)
{
    // 调用外部函数对遥控器数据进行处理，具体见rc_data.h
    rc_input.handle_rc_data(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rc_test");
    ros::NodeHandle nh("~");

    rc_input.init();

    int uav_id = 1;

    //【订阅】PX4遥控器数据
    ros::Subscriber px4_rc_sub =
        nh.subscribe<mavros_msgs::RCIn>("/uav" + std::to_string(uav_id) + "/prometheus/fake_rc_in",
                                        1,
                                        px4_rc_cb);

    // 检查PX4连接状态
    while (ros::ok())
    {
        ros::spinOnce();
        rc_input.printf_info();
        ros::Duration(1.0).sleep();
    }
}