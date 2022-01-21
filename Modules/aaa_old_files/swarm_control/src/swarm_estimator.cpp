#include "swarm_estimator.h"

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    // 读取参数
    nh.param("uav_id", uav_id, 0);
    // 定位数据输入源 0 for vicon, 2 for gazebo ground truth
    nh.param<int>("input_source", input_source, 0);
    //　定位设备偏移量
    nh.param<float>("offset_x", pos_offset[0], 0);
    nh.param<float>("offset_y", pos_offset[1], 0);
    nh.param<float>("offset_z", pos_offset[2], 0);
    nh.param<float>("offset_yaw", yaw_offset, 0);

    uav_name = "/uav" + std::to_string(uav_id);
    msg_name = uav_name + "/control";

    // 变量初始化
    init();

    // 【订阅】无人机当前状态 - 来自飞控
    //  本话题来自飞控(通过Mavros功能包 /plugins/sys_status.cpp)
    state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, state_cb);

    // 【订阅】无人机当前位置 坐标系:ENU系 （此处注意，所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系）
    //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    position_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, pos_cb);

    // 【订阅】无人机当前速度 坐标系:ENU系
    //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 10, vel_cb);

    // 【订阅】无人机当前欧拉角 坐标系:ENU系
    //  本话题来自飞控(通过Mavros功能包 /plugins/imu.cpp读取), 对应Mavlink消息为ATTITUDE (#30), 对应的飞控中的uORB消息为vehicle_attitude.msg
    attitude_sub = nh.subscribe<sensor_msgs::Imu>(uav_name + "/mavros/imu/data", 10, att_cb); 

    // 【订阅】无人机相对高度 此订阅仅针对户外实验
    alt_sub = nh.subscribe<std_msgs::Float64>(uav_name + "/mavros/global_position/rel_alt", 10, alt_cb);

    // 【订阅】mocap估计位置
    mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ uav_name + "/pose", 10, mocap_cb);

    // 【订阅】gazebo仿真真值
    gazebo_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/prometheus/ground_truth", 10, gazebo_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#102), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 100);

    // 【发布】无人机状态合集,包括位置\速度\姿态\模式等,供上层节点使用
    drone_state_pub = nh.advertise<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10);
    
    // 【发布】提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10);

    // 【发布】无人机里程计,用于RVIZ显示
    odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/prometheus/drone_odom", 10);

    // 【发布】无人机运动轨迹
    trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/prometheus/drone_trajectory", 10);

    // 定时器,定时发送vision信息至飞控,保证50Hz以上
    ros::Timer timer_vision_pub = nh.createTimer(ros::Duration(0.02), timercb_vision);

    // 定时器,发布 drone_state_pub,保证20Hz以上
    ros::Timer timer_drone_state_pub = nh.createTimer(ros::Duration(0.05), timercb_drone_state);

    // 定时器,发布 rviz显示,保证1Hz以上
    ros::Timer timer_rviz_pub = nh.createTimer(ros::Duration(0.5), timercb_rviz);

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