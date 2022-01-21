#include "ugv_estimator.h"

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_estimator");
    ros::NodeHandle nh("~");

    // 变量初始化
    init(nh);

    if(sim_mode)
    {
        // 【订阅】gazebo仿真真值
        gazebo_odom_sub = nh.subscribe<nav_msgs::Odometry>(ugv_name + "/prometheus/fake_odom", 1, gazebo_cb);
    }else
    {
        // 【订阅】mocap估计位置
        mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ ugv_name + "/pose", 1, mocap_pos_cb);

        // 【订阅】mocap估计速度
        mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node"+ ugv_name + "/twist", 1, mocap_vel_cb);
    
        // 【订阅】电池状态(无人车底板电压)
        battery_sub = nh.subscribe<std_msgs::Float32>(ugv_name + "/PowerVoltage", 1, battery_cb);
    }

    // 【发布】无人机状态合集,包括位置\速度\姿态\模式等,供上层节点使用
    ugv_state_pub = nh.advertise<prometheus_msgs::UgvState>(ugv_name + "/prometheus/ugv_state", 10);

    // 【发布】无人机里程计
    ugv_odom_pub = nh.advertise<nav_msgs::Odometry>(ugv_name + "/prometheus/ugv_odom", 10);

    // 【发布】无人机运动轨迹
    trajectory_pub = nh.advertise<nav_msgs::Path>(ugv_name + "/prometheus/ugv_trajectory", 10);

    // 【发布】mesh，用于RVIZ显示
    ugv_mesh_pub =  nh.advertise<visualization_msgs::Marker>(ugv_name + "/prometheus/ugv_mesh", 10);

    // 【定时器】发布ugv_state话题
    ros::Timer timer_ugv_state_pub = nh.createTimer(ros::Duration(0.02), timercb_ugv_state);

    // 【定时器】发布RVIZ显示相关话题
    ros::Timer timer_rviz_pub = nh.createTimer(ros::Duration(0.2), timercb_rviz);

    // 频率
    ros::Rate rate(100.0);

    // 等待数据
    ros::Duration(1.0).sleep();

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}