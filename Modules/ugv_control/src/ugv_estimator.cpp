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
        gazebo_odom_sub = nh.subscribe<nav_msgs::Odometry>(ugv_name + "/odom", 1, gazebo_cb);
    }else
    {
        // 【订阅】mocap估计位置
        mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ ugv_name + "/pose", 1, mocap_pos_cb);

        // 【订阅】mocap估计速度
        mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node"+ ugv_name + "/twist", 1, mocap_vel_cb);

        // 【订阅】无人机当前状态 - 来自飞控
        //  本话题来自飞控(通过Mavros功能包 /plugins/sys_status.cpp)
        state_sub = nh.subscribe<mavros_msgs::State>(ugv_name + "/mavros/state", 1, state_cb);

        // 【发布】无人机位置和偏航角 坐标系 ENU系
        //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#102), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
        vision_pub = nh.advertise<geometry_msgs::PoseStamped>(ugv_name + "/mavros/vision_pose/pose", 100);
    }

    // 【发布】无人机状态合集,包括位置\速度\姿态\模式等,供上层节点使用
    ugv_state_pub = nh.advertise<prometheus_msgs::UgvState>(ugv_name + "/prometheus/ugv_state", 10);

    // 【发布】无人机里程计
    ugv_odom_pub = nh.advertise<nav_msgs::Odometry>(ugv_name + "/prometheus/odom", 10);

    // 【发布】无人机运动轨迹
    trajectory_pub = nh.advertise<nav_msgs::Path>(ugv_name + "/prometheus/ugv_trajectory", 10);

    // 定时器,定时发送vision信息至飞控,保证50Hz以上
    ros::Timer timer_vision_pub = nh.createTimer(ros::Duration(0.02), timercb_vision);
    
    // 定时器,发布 ugv_state,保证20Hz以上
    ros::Timer timer_ugv_state_pub = nh.createTimer(ros::Duration(0.05), timercb_ugv_state);

    // 定时器,发布 rviz显示,保证1Hz以上
    ros::Timer timer_rviz_pub = nh.createTimer(ros::Duration(0.5), timercb_rviz);

    // 频率
    ros::Rate rate(100.0);

    sleep(5.0);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}