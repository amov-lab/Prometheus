#include "px4_ego_estimator.h"

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    // 读取参数
    // 无人机名字,即group前缀
    nh.param<string>("uav_name", uav_name, "");
    // 定位数据输入源 0 for vicon, 2 for gazebo ground truth
    nh.param<int>("input_source", input_source, 0);

    msg_name = uav_name + "/control";

    // 变量初始化
    init();

    // 【订阅】Mavros回传
    state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, state_cb);
    position_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, pos_cb);
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 10, vel_cb);
    attitude_sub = nh.subscribe<sensor_msgs::Imu>(uav_name + "/mavros/imu/data", 10, att_cb); 

    // 【订阅】mocap估计位置
    mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ uav_name + "/pose", 10, mocap_cb);

    // 【订阅】gazebo仿真真值
    gazebo_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/prometheus/ground_truth/p300_basic", 10, gazebo_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 100);

    // 【发布】无人机状态合集,包括位置\速度\姿态\模式等,供上层节点使用
    drone_state_pub = nh.advertise<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10);

    // 【发布】无人机里程计
    odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/prometheus/drone_odom", 10);

    // 【发布】无人机运动轨迹
    trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/prometheus/drone_trajectory", 10);

    // 【发布】提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10);

    // 定时器,定时发送vision信息至飞控,保证50Hz以上
    ros::Timer timer_vision_pub = nh.createTimer(ros::Duration(0.02), timercb_vision);

    // 定时器,发布 drone_state_pub,保证20Hz以上
    ros::Timer timer_drone_state_pub = nh.createTimer(ros::Duration(0.05), timercb_drone_state);

    // 定时器,发布 rviz显示,保证1Hz以上
    ros::Timer timer_rviz_pub = nh.createTimer(ros::Duration(1.0), timercb_rviz);

    // 频率
    ros::Rate rate(100.0);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}