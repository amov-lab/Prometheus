#include <gtest/gtest.h>

// ROS
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "darknet_ros_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
