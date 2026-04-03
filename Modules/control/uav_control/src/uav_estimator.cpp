#include "uav_estimator.h"

UAV_estimator::UAV_estimator(ros::NodeHandle &nh): nh(nh)
{
    // 【参数】编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】定位源, 定义见UAVState.msg
    nh.param<int>("control/location_source", location_source, prometheus_msgs::UAVState::GPS);
    // 【参数】最大安全速度
    nh.param<float>("control/maximum_safe_vel_xy", maximum_safe_vel_xy, 4.0f);
    nh.param<float>("control/maximum_safe_vel_z", maximum_safe_vel_z, 3.0f);
    // 【参数】最大vision/px4速度误差
    nh.param<float>("control/maximum_vel_error_for_vision", maximum_vel_error_for_vision, 1.0f);
    // 【参数】最大vision/px4速度误差
    nh.param<float>("control/maximum_vel_error_for_Odom", maximum_vel_error_for_Odom, 0.8f);
 
    // 【参数】D435i tf相对于base_link偏移量
    nh.param<float>("D435i/offset_x", d435i_offset.x, 0.0);
    nh.param<float>("D435i/offset_y", d435i_offset.y, 0.0);
    nh.param<float>("D435i/offset_z", d435i_offset.z, 0.0);
    nh.param<float>("D435i/offset_roll", d435i_offset.roll, 0.0);
    nh.param<float>("D435i/offset_pitch", d435i_offset.pitch, 0.0);
    nh.param<float>("D435i/offset_yaw", d435i_offset.yaw, 0.0);

    // 【参数】lidar tf相对于base_link偏移量
    nh.param<float>("Lidar/offset_x", lidar_offset.x, 0.0);
    nh.param<float>("Lidar/offset_y", lidar_offset.y, 0.0);
    nh.param<float>("Lidar/offset_z", lidar_offset.z, 0.0);
    nh.param<float>("Lidar/offset_roll", lidar_offset.roll, 0.0);
    nh.param<float>("Lidar/offset_pitch", lidar_offset.pitch, 0.0);
    nh.param<float>("Lidar/offset_yaw", lidar_offset.yaw, 0.0);

    // 【参数】T265 tf相对于base_link偏移量
    nh.param<float>("T265/offset_x", t265_offset.x, 0.0);
    nh.param<float>("T265/offset_y", t265_offset.y, 0.0);
    nh.param<float>("T265/offset_z", t265_offset.z, 0.0);
    nh.param<float>("T265/offset_roll", t265_offset.roll, 0.0);
    nh.param<float>("T265/offset_pitch", t265_offset.pitch, 0.0);
    nh.param<float>("T265/offset_yaw", t265_offset.yaw, 0.0);
    
    // 【参数】BSA_SLAM tf相对于base_link偏移量
    nh.param<float>("BSA_SLAM/offset_x", BSA_SLAM_offset.x, 0.0);
    nh.param<float>("BSA_SLAM/offset_y", BSA_SLAM_offset.y, 0.0);
    nh.param<float>("BSA_SLAM/offset_z", BSA_SLAM_offset.z, 0.0);
    nh.param<float>("BSA_SLAM/offset_roll", BSA_SLAM_offset.roll, 0.0);
    nh.param<float>("BSA_SLAM/offset_pitch", BSA_SLAM_offset.pitch, 0.0);
    nh.param<float>("BSA_SLAM/offset_yaw", BSA_SLAM_offset.yaw, 0.0);

    // 【参数】UWB tf相对于A0基站端偏移量
    nh.param<float>("UWB/offset_x", uwb_offset.x, 0.0);
    nh.param<float>("UWB/offset_y", uwb_offset.y, 0.0);
    
    // 【变量】无人机名字
    uav_name = "/uav" + std::to_string(uav_id);
    // 【变量】节点名字
    node_name = "[uav_estimator_uav" + std::to_string(uav_id) + "]";

    // 【订阅】无人机当前状态 - 来自飞控
    px4_state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 1, &UAV_estimator::px4_state_cb, this);

    // 【订阅】无人机电池状态 - 来自飞控
    px4_battery_sub = nh.subscribe<sensor_msgs::BatteryState>(uav_name + "/mavros/battery", 1, &UAV_estimator::px4_battery_cb, this);

    // 【订阅】无人机当前位置 坐标系:ENU系  - 来自飞控
    // 【备注】所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系
    px4_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 1, &UAV_estimator::px4_pos_cb, this);

    // 【订阅】无人机当前速度 坐标系:ENU系 - 来自飞控
    px4_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 1, &UAV_estimator::px4_vel_cb, this);

    // 【订阅】无人机当前欧拉角 坐标系:ENU系 - 来自飞控
    px4_attitude_sub = nh.subscribe<sensor_msgs::Imu>(uav_name + "/mavros/imu/data", 1, &UAV_estimator::px4_att_cb, this);

    // 【订阅】无人机定高雷达数据 - 来自飞控
    px4_range_sub = nh.subscribe<sensor_msgs::Range>(uav_name + "/mavros/distance_sensor/hrlv_ez4_pub", 10, &UAV_estimator::px4_range_cb, this);

    // 根据设定的定位来源订阅不同的定位数据
    if (location_source == prometheus_msgs::UAVState::MOCAP)
    {
        // 【订阅】mocap估计位置
        mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node" + uav_name + "/pose", 1, &UAV_estimator::mocap_cb, this);
    }
    else if (location_source == prometheus_msgs::UAVState::VINS)
    {
        // 【订阅】VINS估计位置
        vins_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vins_estimator/imu_propagate", 1, &UAV_estimator::vins_cb, this);
    }
    else if (location_source == prometheus_msgs::UAVState::T265)
    {
        // 【订阅】T265估计位置
        t265_sub = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 1, &UAV_estimator::t265_cb, this);
    }
    else if (location_source == prometheus_msgs::UAVState::OPTICAL_FLOW)
    {
        // 光流定位直连飞控，飞控积分获得位置，在Prometheus中并不做操作
    }
    else if (location_source == prometheus_msgs::UAVState::viobot)
    {
        // 【订阅】viobot估计位置
        viobot_sub = nh.subscribe<nav_msgs::Odometry>("/pr_loop/odometry_rect", 1, &UAV_estimator::viobot_cb, this);
    }
    else if (location_source == prometheus_msgs::UAVState::MID360)
    {
        // 【订阅】MID360估计位置    
        mid360_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, &UAV_estimator::mid360_cb, this);
    }
    else if (location_source == prometheus_msgs::UAVState::GAZEBO)
    {
        // 【订阅】gazebo仿真真值
        gazebo_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/prometheus/ground_truth", 1, &UAV_estimator::gazebo_cb, this);
    }
    else if (location_source == prometheus_msgs::UAVState::FAKE_ODOM)
    {
        // 【订阅】fake odom
        fake_odom_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/prometheus/fake_odom", 10, &UAV_estimator::fake_odom_cb, this);
    }
    else if (location_source == prometheus_msgs::UAVState::GPS || location_source == prometheus_msgs::UAVState::RTK)
    {
        // 【订阅】GPS状态，来自飞控
        gps_status_sub = nh.subscribe<mavros_msgs::GPSRAW>(uav_name + "/mavros/gpsstatus/gps1/raw", 10, &UAV_estimator::gps_status_cb, this);
        // 【订阅】无人机当前经纬度，来自飞控
        px4_global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>(uav_name + "/mavros/global_position/global", 1, &UAV_estimator::px4_global_pos_cb, this);
        // 【订阅】无人机当前真实高度，来自飞控
        px4_rel_alt_sub = nh.subscribe<std_msgs::Float64>(uav_name + "/mavros/global_position/rel_alt", 1, &UAV_estimator::px4_global_rel_alt_cb, this);
        // 【订阅】设置ENU坐标系下无人机的位置偏移量  坐标系:ENU系 - 来自地面站/终端窗口
        set_local_pose_offset_sub = nh.subscribe<prometheus_msgs::GPSData>(uav_name + "/prometheus/set_local_offset_pose", 1, &UAV_estimator::set_local_pose_offset_cb, this);
        // 【发布】ENU坐标系下的位置偏移量
        local_pose_offset_pub = nh.advertise<prometheus_msgs::OffsetPose>(uav_name + "/prometheus/offset_pose", 1);
        // 【订阅】GPS卫星数量
        gps_satellites_sub = nh.subscribe<std_msgs::UInt32>(uav_name + "/mavros/global_position/raw/satellites", 1, &UAV_estimator::gps_satellites_cb, this);
    }
    else if (location_source == prometheus_msgs::UAVState::UWB)
    {
        // 【订阅】UWB
        uwb_sub = nh.subscribe<prometheus_msgs::LinktrackNodeframe2>("/nlink_linktrack_nodeframe2", 10, &UAV_estimator::uwb_cb, this);
    }
    else if (location_source == prometheus_msgs::UAVState::BSA_SLAM)
    {
        // 【订阅】BSA_SLAM估计位置
        BSA_SLAM_sub = nh.subscribe<nav_msgs::Odometry>("/BSAslam/odometry", 1, &UAV_estimator::BSA_SLAM_cb, this);
    }
    else
    {
        text_info.MessageType = prometheus_msgs::TextInfo::WARN;
        text_info.Message = node_name + ": wrong location_source param, no external location information input!";
        cout << YELLOW << node_name << ": wrong location_source param, no external location information input!" << TAIL << endl;
    }
    // 【订阅】地面站修改ROS参数
    ros_param_set_sub = nh.subscribe<prometheus_msgs::ParamSettings>(uav_name + "/prometheus/param_settings", 1, &UAV_estimator::param_set_cb, this);

    // 【发布】无人机状态合集,包括位置\速度\姿态\模式等,供上层节点使用
    uav_state_pub = nh.advertise<prometheus_msgs::UAVState>(uav_name + "/prometheus/state", 1);

    // 【发布】无人机位置和偏航角，传输至PX4_EKF2模块用于位置姿态估计 坐标系 ENU系
    px4_vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 1);

    // 【发布】无人机里程计,主要用于RVIZ显示
    uav_odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/prometheus/odom", 1);

    // 【发布】相机里程计,主要用于RVIZ显示
    camera_odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/prometheus/camera_odom", 1);

    // 【发布】无人机运动轨迹,主要用于RVIZ显示
    uav_trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/prometheus/trajectory", 1);

    // 【发布】无人机位置(带图标),用于RVIZ显示
    uav_mesh_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/prometheus/uav_mesh", 1);

    // 【发布】运行状态信息(-> 通信节点 -> 地面站)
    ground_station_info_pub = nh.advertise<prometheus_msgs::TextInfo>("/uav" + std::to_string(uav_id) + "/prometheus/text_info", 1);

    if (location_source == prometheus_msgs::UAVState::MOCAP || location_source == prometheus_msgs::UAVState::BSA_SLAM || location_source == prometheus_msgs::UAVState::T265 || location_source == prometheus_msgs::UAVState::viobot || location_source == prometheus_msgs::UAVState::GAZEBO || location_source == prometheus_msgs::UAVState::UWB|| location_source == prometheus_msgs::UAVState::VINS || location_source == prometheus_msgs::UAVState::MID360)
    {
        // 【定时器】当需要使用外部定位设备时，需要定时发送vision信息至飞控,并保证一定频率
        timer_px4_vision_pub = nh.createTimer(ros::Duration(0.02), &UAV_estimator::timercb_pub_vision_pose, this);
    }

    // 【定时器】定时发布 uav_state, uav_odom 保证50Hz以上
    timer_uav_state_pub = nh.createTimer(ros::Duration(0.02), &UAV_estimator::timercb_pub_uav_state, this);

    // 【定时器】定时发布 rviz显示,保证1Hz以上
    timer_rviz_pub = nh.createTimer(ros::Duration(0.05), &UAV_estimator::timercb_rviz, this);

    this->ground_station_info_timer = nh.createTimer(ros::Duration(0.1), &UAV_estimator::sendStationTextInfo, this);

    // 变量初始化
    uav_state.uav_id = uav_id;
    uav_state.connected = false;
    uav_state.armed = false;
    uav_state.mode = "";
    uav_state.location_source = location_source;
    uav_state.odom_valid = false;
    uav_state.gps_status = prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_GPS;
    uav_state.gps_num = 0;
    uav_state.position[0] = 0.0;
    uav_state.position[1] = 0.0;
    uav_state.position[2] = 0.0;
    //该经纬度为阿木实验室测试场地(小花园)的经纬度
    uav_state.latitude = 30.7852600;
    uav_state.longitude = 103.8610300;
    uav_state.altitude = 100.0;
    uav_state.velocity[0] = 0.0;
    uav_state.velocity[1] = 0.0;
    uav_state.velocity[2] = 0.0;
    uav_state.attitude_q.w = 1.0;
    uav_state.attitude_q.x = 0.0;
    uav_state.attitude_q.y = 0.0;
    uav_state.attitude_q.z = 0.0;
    uav_state.attitude[0] = 0.0;
    uav_state.attitude[1] = 0.0;
    uav_state.attitude[2] = 0.0;
    uav_state.attitude_rate[0] = 0.0;
    uav_state.attitude_rate[1] = 0.0;
    uav_state.attitude_rate[2] = 0.0;
    uav_state.battery_state = 0.0;
    uav_state.battery_percetage = 0.0;

    offset_pose.x = 0.0;
    offset_pose.y = 0.0;

    // 【函数】打印参数
    printf_param();
    cout << GREEN << node_name << " init! " << TAIL << endl;

    // 地面站消息打印
    text_info.MessageType = prometheus_msgs::TextInfo::INFO;
    text_info.Message = node_name + " init.";
}

void UAV_estimator::timercb_pub_uav_state(const ros::TimerEvent &e)
{
    if (!uav_state_update)
    {
        // 发布uav_state
        uav_state.header.stamp = ros::Time::now();
        uav_state.odom_valid = false;
        uav_state_pub.publish(uav_state);
        return;
    }

    // 无人机状态检查：
    // 1，检查odom状态
    // 2，待补充
    check_uav_state();

    // 发布uav_state
    uav_state.header.stamp = ros::Time::now();
    uav_state_pub.publish(uav_state);

    if(uav_state.odom_valid)
    {
        // 发布无人机当前odometry(有些节点需要Odometry这个数据类型)
        uav_odom.header.stamp = ros::Time::now();
        uav_odom.header.frame_id = "world";
        uav_odom.child_frame_id = uav_name + "/base_link";
        uav_odom.pose.pose.position.x = uav_state.position[0];
        uav_odom.pose.pose.position.y = uav_state.position[1];
        uav_odom.pose.pose.position.z = uav_state.position[2];
        // 导航算法规定 高度不能小于0
        if (uav_odom.pose.pose.position.z <= 0)
        {
            uav_odom.pose.pose.position.z = 0.01;
        }
        uav_odom.pose.pose.orientation = uav_state.attitude_q;
        uav_odom.twist.twist.linear.x = uav_state.velocity[0];
        uav_odom.twist.twist.linear.y = uav_state.velocity[1];
        uav_odom.twist.twist.linear.z = uav_state.velocity[2];
        uav_odom_pub.publish(uav_odom);

        // 发布无人机d435i当前odometry(有些节点需要Odometry这个数据类型)
        camera_odom.header.frame_id = "world";
        camera_odom.header.stamp = ros::Time::now();
        camera_odom.child_frame_id = uav_name + "/camera_link";
        camera_odom.pose.pose.position.x = uav_odom.pose.pose.position.x + d435i_offset.x;
        camera_odom.pose.pose.position.y = uav_odom.pose.pose.position.y + d435i_offset.y;
        camera_odom.pose.pose.position.z = uav_odom.pose.pose.position.z + d435i_offset.z;

        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::convert(uav_odom.pose.pose.orientation , q_orig);

        q_rot.setRPY(d435i_offset.roll,d435i_offset.pitch,d435i_offset.yaw);//求得 tf 的旋转四元数

        q_new = q_orig*q_rot;  // 通过 姿态的四元数 乘以旋转的四元数 即为 旋转 后的  四元数
        q_new.normalize(); // 归一化

        //  将 旋转后的 tf 四元数 转换 为 msg 四元数
        tf2::convert(q_new, camera_odom.pose.pose.orientation);
    
        camera_odom.twist.twist.linear = uav_odom.twist.twist.linear;
        camera_odom_pub.publish(camera_odom);
    }
}

void UAV_estimator::timercb_pub_vision_pose(const ros::TimerEvent &e)
{
    if (location_source == prometheus_msgs::UAVState::GAZEBO)
    {
        vision_pose = gazebo_pose;
    }
    else if (location_source == prometheus_msgs::UAVState::MOCAP)
    {
        vision_pose = mocap_pose;
    }
    else if (location_source == prometheus_msgs::UAVState::T265)
    {
        vision_pose = t265_pose;
    }
    else if (location_source == prometheus_msgs::UAVState::viobot)
    {

        vision_pose.header.stamp = ros::Time::now();
        vision_pose.pose.position.x = viobot_pose.pose.position.x;
        vision_pose.pose.position.y = viobot_pose.pose.position.y;
        vision_pose.pose.position.z = viobot_pose.pose.position.z;
        vision_pose.pose.orientation.x = q_viobot.x();
        vision_pose.pose.orientation.y = q_viobot.y();
        vision_pose.pose.orientation.z = q_viobot.z();
        vision_pose.pose.orientation.w = q_viobot.w();

    }
    else if (location_source == prometheus_msgs::UAVState::MID360)
    {
        vision_pose = mid360_pose;
    }
    else if (location_source == prometheus_msgs::UAVState::UWB)
    {
 	//j作为计数变量，初始值为0，这里循环6次为了排除初始UWB位置值不准，以第6次为准，测试下来在LinkTrack S型号UWB是没有问题的
        if(j<6)
        {
        
        uwb_offset.x = pos_drone_uwb[0];
        uwb_offset.y = pos_drone_uwb[1];
        j+=1;
        
        }

        vision_pose.header = uav_state.header;
	vision_pose.header.stamp = ros::Time::now();
        // vision_pose = uwb_pose;
        //这里如果不减去初始位置offset值，按照UWB的坐标系，地面电脑A0基站位置是原点，用户可能产生混乱
        //因为无人机基本都是按照上电位置为原点（0，0，0），所以这样减去。如果不需要，那么不减去直接赋值UWB就可以了
        //这里之所以只针对xy做offset偏移，z不做，是因为在LinkTrack S下z轴高度数据是激光雷达提供
        vision_pose.pose.position.x = pos_drone_uwb[0] - uwb_offset.x;
        vision_pose.pose.position.y = pos_drone_uwb[1] - uwb_offset.y;
        vision_pose.pose.position.z = uav_state.range;

        vision_pose.pose.orientation.x = q_uwb.x();
        vision_pose.pose.orientation.y = q_uwb.y();
        vision_pose.pose.orientation.z = q_uwb.z();
        vision_pose.pose.orientation.w = q_uwb.w();
    }
    else if (location_source == prometheus_msgs::UAVState::VINS)
    {
        vision_pose = vins_pose;
    }
    else if (location_source == prometheus_msgs::UAVState::BSA_SLAM)
    {
        vision_pose = BSA_SLAM_pose;
    }
    else
    {
        vision_pose_error = true;
        return;
    }

    Eigen::Vector3d pos_vision = Eigen::Vector3d(vision_pose.pose.position.x, vision_pose.pose.position.y, vision_pose.pose.position.z);
    Eigen::Vector3d pos_px4 = Eigen::Vector3d(uav_state.position[0], uav_state.position[1], uav_state.position[2]);

    // vision位置这一时刻和上一时刻位置偏差如果过大，说明可能视觉定位源发散，对比值需要根据实际测试获得
    if ((pos_vision - last_pos_vision).norm() > maximum_vel_error_for_Odom)
    {
        Odom_pose_error = true;
    }
    else
    {
        Odom_pose_error = false;
    }

    // vision位置和px4回传位置相差较多，一般是PX4中EKF2参数设置错误导致PX4没有收到vision定位数据导致
    // 无人机发生剧烈位移，也会出现本错误，这个需要根据实际测试结果来确定
    if ((pos_vision - pos_px4).norm() > maximum_vel_error_for_vision)
    {
        vision_pose_error = true;
    }
    else
    {
        vision_pose_error = false;
    }

    last_pos_vision = pos_vision;

    if (location_source == prometheus_msgs::UAVState::MID360) {

        // 检查位置协方差
        bool pose_error = (pose_cov[0] > 0.05) ||   // x方差
                        (pose_cov[7] > 0.05) ||   // y方差
                        (pose_cov[14] > 0.05);    // z方差

        // 检查速度协方差
        bool twist_error = (twist_cov[0] > 0.2) ||  // vx方差
                        (twist_cov[7] > 0.2);     // vy方差

        // 更新标志位
        covariance_error = pose_error || twist_error;

    }

    px4_vision_pose_pub.publish(vision_pose);
}

void UAV_estimator::timercb_rviz(const ros::TimerEvent &e)
{
    if(!uav_state.odom_valid)
    {
        return;
    }
    
    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped uav_pos;
    uav_pos.header.stamp = ros::Time::now();
    uav_pos.header.frame_id = "world";
    uav_pos.pose.position.x = uav_state.position[0];
    uav_pos.pose.position.y = uav_state.position[1];
    uav_pos.pose.position.z = uav_state.position[2];
    uav_pos.pose.orientation = uav_state.attitude_q;
    pos_vector.insert(pos_vector.begin(), uav_pos);
    if (pos_vector.size() > TRA_WINDOW)
    {
        pos_vector.pop_back();
    }
    
    nav_msgs::Path uav_trajectory;
    uav_trajectory.header.stamp = ros::Time::now();
    uav_trajectory.header.frame_id = "world";
    uav_trajectory.poses = pos_vector;
    uav_trajectory_pub.publish(uav_trajectory);

    // 发布无人机marker
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = "world";
    meshROS.header.stamp = ros::Time::now();
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = uav_state.position[0];
    meshROS.pose.position.y = uav_state.position[1];
    meshROS.pose.position.z = uav_state.position[2];
    meshROS.pose.orientation.w = uav_state.attitude_q.w;
    meshROS.pose.orientation.x = uav_state.attitude_q.x;
    meshROS.pose.orientation.y = uav_state.attitude_q.y;
    meshROS.pose.orientation.z = uav_state.attitude_q.z;
    meshROS.scale.x = 1.0;
    meshROS.scale.y = 1.0;
    meshROS.scale.z = 1.0;
    meshROS.color.a = 1.0;
    meshROS.color.r = 0.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 1.0;
    meshROS.mesh_resource = std::string("package://prometheus_uav_control/meshes/hummingbird.mesh");
    uav_mesh_pub.publish(meshROS);

    // 发布TF用于RVIZ显示（用于lidar）
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;

    //  |----头设置
    tfs.header.frame_id = "world";       //相对于世界坐标系
    tfs.header.stamp = ros::Time::now(); //时间戳
    //  |----坐标系 ID
    tfs.child_frame_id = uav_name + "/base_link"; //子坐标系，无人机的坐标系
    // tfs.child_frame_id = "/lidar_link"; //子坐标系，无人机的坐标系
    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = uav_state.position[0];
    tfs.transform.translation.y = uav_state.position[1];
    tfs.transform.translation.z = uav_state.position[2];
    //  |--------- 四元数设置
    tfs.transform.rotation.x = uav_state.attitude_q.x;
    tfs.transform.rotation.y = uav_state.attitude_q.y;
    tfs.transform.rotation.z = uav_state.attitude_q.z;
    tfs.transform.rotation.w = uav_state.attitude_q.w;

    //  |--------- 广播器发布数据
    broadcaster.sendTransform(tfs);

    //  |----头设置
    tfs.header.frame_id = uav_name + "/base_link";       //相对于世界坐标系
    tfs.header.stamp = ros::Time::now(); //时间戳
    //  |----坐标系 ID
    tfs.child_frame_id = uav_name + "/lidar_link"; //子坐标系，无人机的坐标系
    // tfs.child_frame_id = "/lidar_link"; //子坐标系，无人机的坐标系
    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = lidar_offset.x;
    tfs.transform.translation.y = lidar_offset.y;
    tfs.transform.translation.z = lidar_offset.z;
    //  |--------- 四元数设置
    tfs.transform.rotation.x = 0.00;
    tfs.transform.rotation.y = 0.00;
    tfs.transform.rotation.z = 0.00;
    tfs.transform.rotation.w = 1.00;

    //q_orig  是原姿态转换的tf的四元数
    //q_rot   旋转四元数
    //q_new   旋转后的姿态四元数
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(tfs.transform.rotation , q_orig);

    q_rot.setRPY(lidar_offset.roll, lidar_offset.pitch, lidar_offset.yaw);//求得 tf 的旋转四元数

    q_new = q_orig*q_rot;  // 通过 姿态的四元数 乘以旋转的四元数 即为 旋转 后的  四元数
    q_new.normalize(); // 归一化

    //  将 旋转后的 tf 四元数 转换 为 msg 四元数
    tf2::convert(q_new, tfs.transform.rotation);

    //  |--------- 广播器发布数据
    broadcaster.sendTransform(tfs);

    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = d435i_offset.x;
    tfs.transform.translation.y = d435i_offset.y;
    tfs.transform.translation.z = d435i_offset.z;

    //  |--------- 四元数设置
    tfs.transform.rotation.x = 0.00;
    tfs.transform.rotation.y = 0.00;
    tfs.transform.rotation.z = 0.00;
    tfs.transform.rotation.w = 1.00;

    tf2::convert(tfs.transform.rotation , q_orig);

    q_rot.setRPY(d435i_offset.roll, d435i_offset.pitch, d435i_offset.yaw);//求得 tf 的旋转四元数

    q_new = q_orig*q_rot;  // 通过 姿态的四元数 乘以旋转的四元数 即为 旋转 后的  四元数
    q_new.normalize(); // 归一化

    //  将 旋转后的 tf 四元数 转换 为 msg 四元数
    tf2::convert(q_new, tfs.transform.rotation);

    tfs.child_frame_id = uav_name + "/camera_link"; //子坐标系，无人机的坐标系
    // tfs.child_frame_id = "/camera_link"; //子坐标系，无人机的坐标系
    //  |--------- 广播器发布数据
    broadcaster.sendTransform(tfs);

    if(location_source == prometheus_msgs::UAVState::T265)
    {
        //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
        tfs.transform.translation.x = t265_offset.x;
        tfs.transform.translation.y = t265_offset.y;
        tfs.transform.translation.z = t265_offset.z;

        //  |--------- 四元数设置
        tfs.transform.rotation.x = 0.00;
        tfs.transform.rotation.y = 0.00;
        tfs.transform.rotation.z = 0.00;
        tfs.transform.rotation.w = 1.00;

        tf2::convert(tfs.transform.rotation , q_orig);

        q_rot.setRPY(t265_offset.roll, t265_offset.pitch, t265_offset.yaw);//求得 tf 的旋转四元数

        q_new = q_orig*q_rot;  // 通过 姿态的四元数 乘以旋转的四元数 即为 旋转 后的  四元数
        q_new.normalize(); // 归一化

        //  将 旋转后的 tf 四元数 转换 为 msg 四元数
        tf2::convert(q_new, tfs.transform.rotation);

        tfs.child_frame_id = "/t265_link"; //子坐标系，无人机的坐标系
        // tfs.child_frame_id = "/camera_link"; //子坐标系，无人机的坐标系
        //  |--------- 广播器发布数据
        broadcaster.sendTransform(tfs);
    }
    else if(location_source == prometheus_msgs::UAVState::BSA_SLAM)
    {
        //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
        tfs.transform.translation.x = BSA_SLAM_offset.x;
        tfs.transform.translation.y = BSA_SLAM_offset.y;
        tfs.transform.translation.z = BSA_SLAM_offset.z;

        //  |--------- 四元数设置
        tfs.transform.rotation.x = 0.00;
        tfs.transform.rotation.y = 0.00;
        tfs.transform.rotation.z = 0.00;
        tfs.transform.rotation.w = 1.00;

        tf2::convert(tfs.transform.rotation , q_orig);

        q_rot.setRPY(BSA_SLAM_offset.roll, BSA_SLAM_offset.pitch, BSA_SLAM_offset.yaw);//求得 tf 的旋转四元数

        q_new = q_orig*q_rot;  // 通过 姿态的四元数 乘以旋转的四元数 即为 旋转 后的  四元数
        q_new.normalize(); // 归一化

        //  将 旋转后的 tf 四元数 转换 为 msg 四元数
        tf2::convert(q_new, tfs.transform.rotation);

        tfs.child_frame_id = "/BSA_SLAM_link"; //子坐标系，无人机的坐标系
        // tfs.child_frame_id = "/camera_link"; //子坐标系，无人机的坐标系
        //  |--------- 广播器发布数据
        broadcaster.sendTransform(tfs);
    }
}

void UAV_estimator::px4_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_state.connected = msg->connected;
    uav_state.armed = msg->armed;
    uav_state.mode = msg->mode;
}

void UAV_estimator::px4_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // 统一坐标系（RTK情况下，可以设置offset_pose，其他情况offset_pose为零）
    uav_state.position[0] = msg->pose.position.x + offset_pose.x;
    uav_state.position[1] = msg->pose.position.y + offset_pose.y;
    uav_state.position[2] = msg->pose.position.z;
    uav_state_update = true;
}

void UAV_estimator::px4_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    uav_state.latitude = msg->latitude;
    uav_state.longitude = msg->longitude;
    uav_state.altitude = msg->altitude;
}

void UAV_estimator::px4_global_rel_alt_cb(const std_msgs::Float64::ConstPtr &msg)
{
    uav_state.rel_alt = msg->data;
}

void UAV_estimator::px4_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    uav_state.velocity[0] = msg->twist.linear.x;
    uav_state.velocity[1] = msg->twist.linear.y;
    uav_state.velocity[2] = msg->twist.linear.z;
}

void UAV_estimator::px4_att_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);

    uav_state.attitude_q = msg->orientation;
    uav_state.attitude[0] = euler_fcu[0];
    uav_state.attitude[1] = euler_fcu[1];
    uav_state.attitude[2] = euler_fcu[2];
    uav_state.attitude_rate[0] = msg->angular_velocity.x;
    uav_state.attitude_rate[1] = msg->angular_velocity.y;
    uav_state.attitude_rate[2] = msg->angular_velocity.z;
}

void UAV_estimator::px4_battery_cb(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    uav_state.battery_state = msg->voltage;
    uav_state.battery_percetage = msg->percentage;
}

void UAV_estimator::px4_range_cb(const sensor_msgs::Range::ConstPtr &msg)
{
    uav_state.range = msg->range;
}

void UAV_estimator::gps_status_cb(const mavros_msgs::GPSRAW::ConstPtr &msg)
{
    uav_state.gps_status = msg->fix_type;
    get_gps_stamp = ros::Time::now();
}

void UAV_estimator::mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    mocap_pose = *msg;
    get_mocap_stamp = ros::Time::now(); // 记录时间戳，防止超时
}

void UAV_estimator::vins_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    vins_pose = *msg;
    get_vins_stamp = ros::Time::now(); // 记录时间戳，防止超时
}

void UAV_estimator::gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    gazebo_pose.header = msg->header;
    gazebo_pose.pose = msg->pose.pose;
    get_gazebo_stamp = ros::Time::now(); // 记录时间戳，防止超时
    // cout << YELLOW << "get_gazebo_stamp:[ " << (get_gazebo_stamp).toSec() << " ] s" << TAIL << endl;
}

void UAV_estimator::uwb_cb(const prometheus_msgs::LinktrackNodeframe2::ConstPtr &msg)
{
    pos_drone_uwb[0] = msg->pos_3d[0];
    pos_drone_uwb[1] = msg->pos_3d[1];
    pos_drone_uwb[2] = msg->pos_3d[2];
    q_uwb = Eigen::Quaterniond(msg->quaternion[0],msg->quaternion[1],msg->quaternion[2],msg->quaternion[3]);
    Euler_uwb = quaternion_to_euler(q_uwb);

    get_uwb_stamp = ros::Time::now(); // 记录时间戳，防止超时
}

void UAV_estimator::t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    t265_pose.header = msg->header;
    t265_pose.pose = msg->pose.pose;
    get_t265_stamp = ros::Time::now(); // 记录时间戳，防止超时
}

void UAV_estimator::BSA_SLAM_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    BSA_SLAM_pose.header = msg->header;
    BSA_SLAM_pose.pose = msg->pose.pose;
    get_BSA_SLAM_stamp = ros::Time::now(); // 记录时间戳，防止超时
}

void UAV_estimator::viobot_cb(const nav_msgs::Odometry::ConstPtr &msg) 
{
    viobot_pose.header = msg->header;
    viobot_pose.pose = msg->pose.pose;

    q_viobot = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );

    Eigen::Quaterniond rotation1(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond rotation2(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

    q_viobot = q_viobot * rotation1 * rotation2;

    Euler_viobot = quaternion_to_euler(q_viobot);

    get_viobot_stamp = ros::Time::now(); // 记录时间戳，防止超时
}

void UAV_estimator::mid360_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    mid360_pose.header.frame_id = msg->header.frame_id;
    mid360_pose.header.stamp = ros::Time::now();
    mid360_pose.pose = msg->pose.pose;

    // 提取协方差矩阵
    pose_cov = msg->pose.covariance;
    twist_cov = msg->twist.covariance;

    get_mid360_stamp = ros::Time::now(); // 记录时间戳，防止超时
   
}
void UAV_estimator::fake_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // uav_state 直接赋值
    uav_state.connected = true;
    uav_state.armed = true;
    uav_state.odom_valid = true;
    uav_state.mode = "OFFBOARD";
    uav_state.position[0] = msg->pose.pose.position.x;
    uav_state.position[1] = msg->pose.pose.position.y;
    uav_state.position[2] = msg->pose.pose.position.z;
    uav_state.velocity[0] = msg->twist.twist.linear.x;
    uav_state.velocity[1] = msg->twist.twist.linear.y;
    uav_state.velocity[2] = msg->twist.twist.linear.z;
    uav_state.attitude_q = msg->pose.pose.orientation;
    Eigen::Quaterniond q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d euler = quaternion_to_euler(q);
    uav_state.attitude[0] = euler[0];
    uav_state.attitude[1] = euler[1];
    uav_state.attitude[2] = euler[2];
    uav_state.attitude_rate[0] = 0.0;
    uav_state.attitude_rate[1] = 0.0;
    uav_state.attitude_rate[2] = 0.0;

    uav_state_update = true;
}

void UAV_estimator::check_uav_state()
{
    if (uav_state.connected == false)
    {
        return;
    }
    // 检查odom状态
    odom_state = check_uav_odom();
    if(odom_first_check)
    {
        last_odom_state = odom_state;
        odom_first_check = false;
    }

    if (odom_state == 1 && last_odom_state != 1)
    {
        text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
        text_info.Message = "Odom invalid: Get Vision Pose Timeout!";
        cout << RED << node_name << "--->  Odom invalid: Get Vision Pose Timeout! " << TAIL << endl;
    }
    else if (odom_state == 2 && last_odom_state != 2)
    {
        text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
        text_info.Message = "Odom invalid: Velocity too large!";

        cout << RED << node_name << "--->  Odom invalid: Velocity too large! " << TAIL << endl;
    }
    else if (odom_state == 3 && last_odom_state != 3)
    {
        text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
        text_info.Message = "Odom invalid: vision_pose_error!";
        cout << RED << node_name << "--->  Odom invalid: vision_pose_error! " << TAIL << endl;
    }
    else if (odom_state == 10 && last_odom_state != 10)
    {
        text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
        text_info.Message = "Odom invalid: Odom_vision_pose_error!";
        cout << RED << node_name << "--->  Odom invalid: Odom_vision_pose_error! " << TAIL << endl;
    }
    else if (odom_state == 11 && last_odom_state != 11)
    {
        text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
        text_info.Message = "Odom invalid: MID360 covariance error!";
        cout << RED << node_name << "--->  Odom invalid: MID360 covariance error! " << TAIL << endl;
    }
    else if (odom_state == 4 && last_odom_state != 4)
    {
        text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
        text_info.Message = "Odom invalid: GPS/RTK location error!";
        cout << RED << node_name << "--->  Odom invalid: GPS/RTK location error! " << TAIL << endl;
    }
    else if (odom_state == 5 && last_odom_state != 5)
    {
        if(location_source == prometheus_msgs::UAVState::GPS)
        {
            text_info.MessageType = prometheus_msgs::TextInfo::WARN;
            text_info.Message = "The GPS status seems to be RTK, please confirm whether the location source selection is normal!";
            cout << YELLOW << node_name << "--->  The GPS status seems to be RTK, please confirm whether the location source selection is normal!" << TAIL << endl;
        }
        
        if(location_source == prometheus_msgs::UAVState::RTK)
        {
            text_info.MessageType = prometheus_msgs::TextInfo::WARN;
            text_info.Message = "Odom invalid: RTK not fixed!";
            cout << YELLOW << node_name << "--->  Odom invalid: RTK not fixed! " << TAIL << endl;
        }

    }
    else if (odom_state == 6 && last_odom_state != 6)
    {
        text_info.MessageType = prometheus_msgs::TextInfo::WARN;
        text_info.Message = "Odom invalid: Get UWB Pose Timeout!";
        cout << YELLOW << node_name << "--->  Odom invalid: Get UWB Pose Timeout! " << TAIL << endl;
    }
    else if (odom_state == 7 && last_odom_state != 7)
    {
        text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
        text_info.Message = "Odom invalid: Get GPS/RTK Pose Timeout!";
        cout << YELLOW << node_name << "--->  Odom invalid: Get GPS/RTK Pose Timeout! " << TAIL << endl;
    }

    if (odom_state == 9 || odom_state == 5)
    {
        uav_state.odom_valid = true;
    }
    else
    {
        uav_state.odom_valid = false;
    }
    last_odom_state = odom_state;
}

int UAV_estimator::check_uav_odom()
{
    ros::Time time_now = ros::Time::now();

    // odom失效可能原因1：外部定位数据接收超时
    if (location_source == prometheus_msgs::UAVState::GAZEBO && (time_now - get_gazebo_stamp).toSec() > GAZEBO_TIMEOUT)
    {
        return 1;
    }
    else if (location_source == prometheus_msgs::UAVState::MOCAP && (time_now - get_mocap_stamp).toSec() > MOCAP_TIMEOUT)
    {
        return 1;
    }
    else if (location_source == prometheus_msgs::UAVState::T265 && (time_now - get_t265_stamp).toSec() > T265_TIMEOUT)
    {
        return 1;
    }
    else if (location_source == prometheus_msgs::UAVState::viobot && (time_now - get_viobot_stamp).toSec() > VIOBOT_TIMEOUT)
    {
        return 1;
    }
    else if (location_source == prometheus_msgs::UAVState::MID360 && (time_now - get_mid360_stamp).toSec() > MID360_TIMEOUT)
    {
        return 1;
    }
    // UWB
    else if (location_source == prometheus_msgs::UAVState::UWB && (time_now - get_uwb_stamp).toSec() > UWB_TIMEOUT)
    {
        return 1;
    }
    else if (location_source == prometheus_msgs::UAVState::VINS && (time_now - get_vins_stamp).toSec() > VINS_TIMEOUT)
    {
        return 1;
    }
    else if (location_source == prometheus_msgs::UAVState::BSA_SLAM && (time_now - get_BSA_SLAM_stamp).toSec() > BSA_SLAM_TIMEOUT)
    {
        return 1;
    }
    else if ((location_source == prometheus_msgs::UAVState::GPS || location_source == prometheus_msgs::UAVState::RTK) && (time_now - get_gps_stamp).toSec() > GPS_TIMEOUT)
    {
        return 7;
    }
    

    // odom失效可能原因2：无人机合速度过大，认为定位模块失效
    Eigen::Vector2d uav_vel_xy = Eigen::Vector2d(uav_state.velocity[0], uav_state.velocity[1]);
    if (uav_vel_xy.norm() > maximum_safe_vel_xy || uav_state.velocity[2] > maximum_safe_vel_z)
    {
        return 2;
    }

    // odom失效可能原因3：无人机位置与外部定位设备原始值相差过多
    if (vision_pose_error && location_source != prometheus_msgs::UAVState::OPTICAL_FLOW)
    {
        return 3;
    }

    // odom失效可能原因：外部定位设备这一时刻和上一时刻原始值相差过多
    if (Odom_pose_error && location_source != prometheus_msgs::UAVState::OPTICAL_FLOW)
    {
        return 10;
    }

        // mid360:covariance error
    if (location_source == prometheus_msgs::UAVState::MID360 && covariance_error)
    {
        return 11;
    }

    // odom失效可能原因4:GPS定位模块数据异常,无法获取定位数据
    if (location_source == prometheus_msgs::UAVState::GPS)
    {
        if (uav_state.gps_status < prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX)
        {
            return 4;
        }
        else if(uav_state.gps_status > prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX)
        {
            return 5;
        }
    }

    if (location_source == prometheus_msgs::UAVState::RTK)
    {
        if (uav_state.gps_status < prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX)
        {
            return 4;
        }
        // odom数据可信度降低可能原因5:RTK定位精度未达到FIXED状态(非odom失效状态)
        else if (uav_state.gps_status <= prometheus_msgs::UAVState::GPS_FIX_TYPE_RTK_FLOATR)
        {
            return 5;
        }
    }
    return 9;
}

void UAV_estimator::printf_uav_state()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>> UAV [" << uav_id << "] State  <<<<<<<<<<<<<<<<<<<<" << TAIL << endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // 打印 无人机状态
    if (uav_state.connected == true)
    {
        cout << GREEN << "PX4 Status:  [ Connected ] ";
    }
    else
    {
        text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
        text_info.Message = "PX4 unconnected";
        cout << RED << "PX4 Status:[ Unconnected ] ";
    }
    //是否上锁
    if (uav_state.armed == true)
    {
        cout << GREEN << "[  Armed   ] ";
    }
    else
    {
        cout << RED << "[ DisArmed ] ";
    }

    cout << "[ " << uav_state.mode << " ] " << TAIL << endl;

    switch (location_source)
    {
    case prometheus_msgs::UAVState::MOCAP:
        cout << GREEN << "Location: [ MOCAP ] " << TAIL << endl;
        cout << GREEN << "MOCAP_pos [X Y Z] : " << mocap_pose.pose.position.x << " [ m ] " << mocap_pose.pose.position.y << " [ m ] " << mocap_pose.pose.position.z << " [ m ] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::T265:
        cout << GREEN << "Location: [ T265 ] " << TAIL << endl;
        cout << GREEN << "T265_pos [X Y Z] : " << t265_pose.pose.position.x << " [ m ] " << t265_pose.pose.position.y << " [ m ] " << t265_pose.pose.position.z << " [ m ] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::OPTICAL_FLOW:
        cout << GREEN << "Location: [ OPTICAL_FLOW ] " << TAIL << endl;
        cout << GREEN << "OPTICAL_FLOW_pos [X Y Z] : " << uav_state.position[0] << " [ m ] " << uav_state.position[1] << " [ m ] " << uav_state.position[2] << " [ m ] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::viobot:
        cout << GREEN << "Location: [ viobot ] " << TAIL << endl;
        cout << GREEN << "viobot_pos [X Y Z] : " << viobot_pose.pose.position.x << " [ m ] " << viobot_pose.pose.position.y << " [ m ] " << viobot_pose.pose.position.z << " [ m ] " << TAIL << endl;
        cout << GREEN << "viobot_p [W X Y Z ] : " << vision_pose.pose.orientation.w << " [ ] "<< vision_pose.pose.orientation.x << " [ ] " << vision_pose.pose.orientation.y << " [ ] " << vision_pose.pose.orientation.z << " [ ] "  << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::MID360:
        cout << GREEN << "Location: [ MID360 ] " << TAIL << endl;
        cout << GREEN << "MID360_pos [X Y Z] : " << mid360_pose.pose.position.x << " [ m ] " << mid360_pose.pose.position.y << " [ m ] " << mid360_pose.pose.position.z << " [ m ] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::GAZEBO:
        cout << GREEN << "Location: [ GAZEBO ] " << TAIL << endl;
        cout << GREEN << "GAZEBO_pos [X Y Z] : " << gazebo_pose.pose.position.x << " [ m ] " << gazebo_pose.pose.position.y << " [ m ] " << gazebo_pose.pose.position.z << " [ m ] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::FAKE_ODOM:
        cout << GREEN << "Location: [ FAKE_ODOM ] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::GPS:
        cout << GREEN << "Location: [ GPS ] " << TAIL;
        printf_gps_status();
        break;
    case prometheus_msgs::UAVState::RTK:
        cout << GREEN << "Location: [ RTK ] " << TAIL;
        printf_gps_status();
        break;
    case prometheus_msgs::UAVState::UWB:
        cout << GREEN << "Location: [ UWB ] " << TAIL;
        cout << GREEN << " uwb_offset [X Y Z] : " << uwb_offset.x << " [ m ] " << uwb_offset.y << " [ m ] " << TAIL << endl;
        cout << GREEN << "UWB_pos [X Y Z] : " << pos_drone_uwb[0]  << " [ m ] " << pos_drone_uwb[1] << " [ m ] " << pos_drone_uwb[2] << " [ m ] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::VINS:
        cout << GREEN << "Location: [ VINS ] " << TAIL << endl;
        cout << GREEN << "VINS_pos [X Y Z] : " << vins_pose.pose.position.x << " [ m ] " << vins_pose.pose.position.y << " [ m ] " << vins_pose.pose.position.z << " [ m ] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::BSA_SLAM:
        cout << GREEN << "Location: [ BSA_SLAM ] " << TAIL << endl;
        cout << GREEN << "BSA_SLAM_pos [X Y Z] : " << BSA_SLAM_pose.pose.position.x << " [ m ] " << BSA_SLAM_pose.pose.position.y << " [ m ] " << BSA_SLAM_pose.pose.position.z << " [ m ] " << TAIL << endl;
        break;
    }

    if (uav_state.odom_valid)
    {
        cout << GREEN << "Odom Status     : [ Valid ] " << TAIL << endl;
    }
    else
    {
        cout << RED << "Odom Status     : [ Invalid ] " << TAIL << endl;
    }

    cout << GREEN << "UAV_pos [X Y Z] : " << uav_state.position[0] << " [ m ] " << uav_state.position[1] << " [ m ] " << uav_state.position[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "UAV_vel [X Y Z] : " << uav_state.velocity[0] << " [m/s] " << uav_state.velocity[1] << " [m/s] " << uav_state.velocity[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "UAV_att [R P Y] : " << uav_state.attitude[0] * 180 / M_PI << " [deg] " << uav_state.attitude[1] * 180 / M_PI << " [deg] " << uav_state.attitude[2] * 180 / M_PI << " [deg] " << TAIL << endl;

    cout << GREEN << "Battery Voltage : " << uav_state.battery_state << " [V] "
         << "  Battery Percent : " << uav_state.battery_percetage << TAIL << endl;
}

void UAV_estimator::printf_gps_status()
{
    // 确认一下，哪些是红色，哪些是绿色，todo...
    if (location_source == prometheus_msgs::UAVState::GPS)
    {
        switch (uav_state.gps_status)
        {
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_GPS:
            cout << RED << " [GPS_FIX_TYPE_NO_GPS] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_FIX:
            cout << RED << " [GPS_FIX_TYPE_NO_FIX] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_2D_FIX:
            cout << YELLOW << " [GPS_FIX_TYPE_2D_FIX] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX:
            cout << GREEN << " [GPS_FIX_TYPE_3D_FIX] " << TAIL << endl;
            break;
        }
    }
    if (location_source == prometheus_msgs::UAVState::RTK)
    {
        switch (uav_state.gps_status)
        {
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_GPS:
            cout << RED << " [GPS_FIX_TYPE_NO_GPS] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_FIX:
            cout << RED << " [GPS_FIX_TYPE_NO_FIX] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_2D_FIX:
            cout << RED << " [GPS_FIX_TYPE_2D_FIX] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX:
            cout << YELLOW << " [GPS_FIX_TYPE_3D_FIX] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_DGPS:
            cout << YELLOW << " [GPS_FIX_TYPE_DGPS] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_RTK_FLOATR:
            cout << YELLOW << " [GPS_FIX_TYPE_RTK_FLOATR] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_RTK_FIXEDR:
            cout << GREEN << " [GPS_FIX_TYPE_RTK_FIXEDR] " << TAIL << endl;
            break;
        }
        // offset_pose
        cout << GREEN << "offset_pose [X Y] : " << offset_pose.x << " [ m ] " << offset_pose.y << " [ m ] " << TAIL << endl;
    }

    // 确定下单位，todo
    cout << GREEN << "GPS [lat lon alt] : " << uav_state.latitude << " [ deg ] " << uav_state.longitude << " [ deg ] " << uav_state.altitude << " [ m ] " << TAIL << endl;
}

void UAV_estimator::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>> UAV estimator Param <<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << GREEN << "uav_id                        : " << uav_id << " " << TAIL << endl;
    cout << GREEN << "maximum_safe_vel_xy           : " << maximum_safe_vel_xy << " [m/s] " << TAIL << endl;
    cout << GREEN << "maximum_safe_vel_z            : " << maximum_safe_vel_z << " [m/s] " << TAIL << endl;
    cout << GREEN << "maximum_vel_error_for_vision  : " << maximum_vel_error_for_vision << " [m/s] " << TAIL << endl;
    cout << GREEN << "maximum_vel_error_for_Odom    : " << maximum_vel_error_for_Odom << " [m/s] " << TAIL << endl;

    if (location_source == prometheus_msgs::UAVState::GAZEBO)
    {
        cout << GREEN << "location_source: [GAZEBO] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::MOCAP)
    {
        cout << GREEN << "location_source: [MOCAP] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::T265)
    {
        cout << GREEN << "location_source: [T265] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::OPTICAL_FLOW)
    {
        cout << GREEN << "location_source: [OPTICAL_FLOW] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::viobot)
    {
        cout << GREEN << "location_source: [viobot] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::MID360)
    {
        cout << GREEN << "location_source: [MID360] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::FAKE_ODOM)
    {
        cout << GREEN << "location_source: [FAKE_ODOM] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::GPS)
    {
        cout << GREEN << "location_source: [GPS] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::RTK)
    {
        cout << GREEN << "location_source: [RTK] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::UWB)
    {
        cout << GREEN << "location_source: [UWB] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::VINS)
    {
        cout << GREEN << "location_source: [VINS] " << TAIL << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::BSA_SLAM)
    {
        cout << GREEN << "location_source: [BSA_SLAM] " << TAIL << endl;
    }
    else
    {
        cout << GREEN << "location_source: [UNKNOW] " << endl;
    }
}

void UAV_estimator::set_local_pose_offset_cb(const prometheus_msgs::GPSData::ConstPtr &msg)
{
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    Eigen::Vector3d origin_gps;
    Eigen::Vector3d origin_ecef;
    Eigen::Vector3d current_uav_gps;
    Eigen::Vector3d current_uav_ecef;
    Eigen::Vector3d ecef_offset;
    Eigen::Vector3d enu_offset;

    origin_gps[0] = msg->latitude;
    origin_gps[1] = msg->longitude;
    origin_gps[2] = msg->altitude;

    current_uav_gps[0] = uav_state.latitude;
    current_uav_gps[1] = uav_state.longitude;
    current_uav_gps[2] = uav_state.altitude;

    earth.Forward(origin_gps[0], origin_gps[1], origin_gps[2], origin_ecef[0], origin_ecef[1], origin_ecef[2]);
    earth.Forward(current_uav_gps[0], current_uav_gps[1], current_uav_gps[2], current_uav_ecef[0], current_uav_ecef[1], current_uav_ecef[2]);

    ecef_offset = current_uav_ecef - origin_ecef;
    enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, origin_gps);

    // estimator节点根据GPS初始位置解算得到初始偏差，只发布一次
    // 发布出来的uav_state，在uav1的坐标系下
    // 每个飞机的local_position还是在起飞坐标系
    // 所以发布控制指令也要加上偏差
    offset_pose.uav_id = uav_id;
    //重复调用时,避免偏移量错误计算,需要先减掉偏移量
    offset_pose.x += enu_offset[0] - uav_state.position[0] + msg->x;
    offset_pose.y += enu_offset[1] - uav_state.position[1] + msg->y;

    local_pose_offset_pub.publish(offset_pose);
}

void UAV_estimator::gps_satellites_cb(const std_msgs::UInt32::ConstPtr &msg)
{
    uav_state.gps_num = msg->data;
}

void UAV_estimator::param_set_cb(const prometheus_msgs::ParamSettings::ConstPtr &msg)
{
    try
    {
        size_t size = msg->param_name.size();
        for(size_t i = 0; i < size; i++)
        {
            // 如果不包含本节点名则跳过
            if (msg->param_name[i].find("/uav_control_main_" + std::to_string(uav_id) + "/") == std::string::npos)
                continue;

            std::cout << msg->param_name[i] << " : " << msg->param_value[i] << std::endl;

            // 有些参数必须在未解锁前才能修改
            if(msg->param_name[i].find("control/location_source") != std::string::npos)
            {
                if(!uav_state.armed)
                    switch_location_source(location_source, std::stoi(msg->param_value[i]));
            }
            else if(msg->param_name[i].find("control/maximum_safe_vel_xy") != std::string::npos)
                maximum_safe_vel_xy = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("control/maximum_safe_vel_z") != std::string::npos)
                maximum_safe_vel_z = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("control/maximum_vel_error_for_vision") != std::string::npos)
                maximum_vel_error_for_vision = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("control/maximum_vel_error_for_Odom") != std::string::npos)
            maximum_vel_error_for_Odom = std::stod(msg->param_value[i]);
            // d435i offset
            else if(msg->param_name[i].find("D435i/offset_x") != std::string::npos)
                d435i_offset.x = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("D435i/offset_y") != std::string::npos)
                d435i_offset.y = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("D435i/offset_z") != std::string::npos)
                d435i_offset.z = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("D435i/offset_roll") != std::string::npos)
                d435i_offset.roll = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("D435i/offset_pitch") != std::string::npos)
                d435i_offset.pitch = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("D435i/offset_yaw") != std::string::npos)
                d435i_offset.yaw = std::stod(msg->param_value[i]);
            // lidar offset
            else if(msg->param_name[i].find("Lidar/offset_x") != std::string::npos)
                lidar_offset.x = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("Lidar/offset_y") != std::string::npos)
                lidar_offset.y = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("Lidar/offset_z") != std::string::npos)
                lidar_offset.z = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("Lidar/offset_roll") != std::string::npos)
                lidar_offset.roll = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("Lidar/offset_pitch") != std::string::npos)
                lidar_offset.pitch = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("Lidar/offset_yaw") != std::string::npos)
                lidar_offset.yaw = std::stod(msg->param_value[i]);
            // t265
            else if(msg->param_name[i].find("T265/offset_x") != std::string::npos)
                t265_offset.x = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("T265/offset_y") != std::string::npos)
                t265_offset.y = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("T265/offset_z") != std::string::npos)
                t265_offset.z = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("T265/offset_roll") != std::string::npos)
                t265_offset.roll = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("T265/offset_pitch") != std::string::npos)
                t265_offset.pitch = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("T265/offset_yaw") != std::string::npos)
                t265_offset.yaw = std::stod(msg->param_value[i]);
            // BSA_SLAM
            else if(msg->param_name[i].find("BSA_SLAM/offset_x") != std::string::npos)
                BSA_SLAM_offset.x = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("BSA_SLAM/offset_y") != std::string::npos)
                BSA_SLAM_offset.y = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("BSA_SLAM/offset_z") != std::string::npos)
                BSA_SLAM_offset.z = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("BSA_SLAM/offset_roll") != std::string::npos)
                BSA_SLAM_offset.roll = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("BSA_SLAM/offset_pitch") != std::string::npos)
                BSA_SLAM_offset.pitch = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("BSA_SLAM/offset_yaw") != std::string::npos)
                BSA_SLAM_offset.yaw = std::stod(msg->param_value[i]);
            // uwb offset
            else if(msg->param_name[i].find("UWB/offset_x") != std::string::npos)
                uwb_offset.x = std::stod(msg->param_value[i]);
            else if(msg->param_name[i].find("UWB/offset_y") != std::string::npos)
                uwb_offset.y = std::stod(msg->param_value[i]);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

//向地面发送反馈信息,如果重复,将不会发送
void UAV_estimator::sendStationTextInfo(const ros::TimerEvent &e)
{
    if(this->text_info.Message == this->last_text_info.Message)
    {
        return;
    }
    else
    {
        this->text_info.header.stamp = ros::Time::now();
        this->ground_station_info_pub.publish(this->text_info);
        this->last_text_info = this->text_info;
        return;
    }
}

void UAV_estimator::switch_location_source(int old_location_source, int new_location_source)
{
    if(old_location_source == new_location_source) return;

    if(timer_px4_vision_pub.isValid())
    {
        timer_px4_vision_pub.stop();
    }
    
    // 关闭话题
    switch(old_location_source)
    {
    case prometheus_msgs::UAVState::MOCAP:
        mocap_sub.shutdown();
        break;
    case prometheus_msgs::UAVState::T265:
        t265_sub.shutdown();
        break;
    case prometheus_msgs::UAVState::GAZEBO:
        gazebo_sub.shutdown();
        break;
    case prometheus_msgs::UAVState::FAKE_ODOM:
        fake_odom_sub.shutdown();
        break;
    case prometheus_msgs::UAVState::GPS:
    case prometheus_msgs::UAVState::RTK:
        gps_status_sub.shutdown();
        px4_global_position_sub.shutdown();
        px4_rel_alt_sub.shutdown();
        set_local_pose_offset_sub.shutdown();
        local_pose_offset_pub.shutdown();
        gps_satellites_sub.shutdown();
        break;
    case prometheus_msgs::UAVState::UWB:
        uwb_sub.shutdown();
        break;
    case prometheus_msgs::UAVState::VINS:
        vins_sub.shutdown();
        break;
    case prometheus_msgs::UAVState::OPTICAL_FLOW:
        break;
    case prometheus_msgs::UAVState::viobot:
        viobot_sub.shutdown();
        break;
    case prometheus_msgs::UAVState::MID360:
        mid360_sub.shutdown();
        break;
    case prometheus_msgs::UAVState::BSA_SLAM:
        BSA_SLAM_sub.shutdown();
        break;
    }

    location_source = new_location_source;
    uav_state.location_source = location_source;

    switch(new_location_source)
    {
    case prometheus_msgs::UAVState::MOCAP:
        mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node" + uav_name + "/pose", 1, &UAV_estimator::mocap_cb, this);
        break;
    case prometheus_msgs::UAVState::T265:
        t265_sub = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 1, &UAV_estimator::t265_cb, this);
        break;
    case prometheus_msgs::UAVState::GAZEBO:
        gazebo_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/prometheus/ground_truth", 1, &UAV_estimator::gazebo_cb, this);
        break;
    case prometheus_msgs::UAVState::FAKE_ODOM:
        fake_odom_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/prometheus/fake_odom", 10, &UAV_estimator::fake_odom_cb, this);
        break;
    case prometheus_msgs::UAVState::GPS:
    case prometheus_msgs::UAVState::RTK:
        gps_status_sub = nh.subscribe<mavros_msgs::GPSRAW>(uav_name + "/mavros/gpsstatus/gps1/raw", 10, &UAV_estimator::gps_status_cb, this);
        px4_global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>(uav_name + "/mavros/global_position/global", 1, &UAV_estimator::px4_global_pos_cb, this);
        px4_rel_alt_sub = nh.subscribe<std_msgs::Float64>(uav_name + "/mavros/global_position/rel_alt", 1, &UAV_estimator::px4_global_rel_alt_cb, this);
        set_local_pose_offset_sub = nh.subscribe<prometheus_msgs::GPSData>(uav_name + "/prometheus/set_local_offset_pose", 1, &UAV_estimator::set_local_pose_offset_cb, this);
        local_pose_offset_pub = nh.advertise<prometheus_msgs::OffsetPose>(uav_name + "/prometheus/offset_pose", 1);
        gps_satellites_sub = nh.subscribe<std_msgs::UInt32>(uav_name + "/mavros/global_position/raw/satellites", 1, &UAV_estimator::gps_satellites_cb, this);
        break;
    case prometheus_msgs::UAVState::UWB:
        uwb_sub = nh.subscribe<prometheus_msgs::LinktrackNodeframe2>("/nlink_linktrack_nodeframe2", 10, &UAV_estimator::uwb_cb, this);
        break;
    case prometheus_msgs::UAVState::VINS:
        vins_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vins_estimator/imu_propagate", 1, &UAV_estimator::vins_cb, this);
        break;
    case prometheus_msgs::UAVState::OPTICAL_FLOW:
        break;
    case prometheus_msgs::UAVState::viobot:
        viobot_sub = nh.subscribe<nav_msgs::Odometry>("/pr_loop/odometry_rect", 1, &UAV_estimator::viobot_cb, this);
        break;
    case prometheus_msgs::UAVState::MID360:
        mid360_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, &UAV_estimator::mid360_cb, this);
        break;
    case prometheus_msgs::UAVState::BSA_SLAM:
        BSA_SLAM_sub = nh.subscribe<nav_msgs::Odometry>("/BSAslam/odometry", 1, &UAV_estimator::BSA_SLAM_cb, this);
        break;
    default:
        text_info.MessageType = prometheus_msgs::TextInfo::WARN;
        text_info.Message = node_name + ": wrong location_source param, no external location information input!";
        cout << YELLOW << node_name << ": wrong location_source param, no external location information input!" << TAIL << endl;
        break;
    }

    if (location_source == prometheus_msgs::UAVState::MOCAP || location_source == prometheus_msgs::UAVState::BSA_SLAM || location_source == prometheus_msgs::UAVState::T265 || location_source == prometheus_msgs::UAVState::viobot || location_source == prometheus_msgs::UAVState::GAZEBO || location_source == prometheus_msgs::UAVState::UWB|| location_source == prometheus_msgs::UAVState::VINS || location_source == prometheus_msgs::UAVState::MID360)
    {
        // 【定时器】当需要使用外部定位设备时，需要定时发送vision信息至飞控,并保证一定频率
        timer_px4_vision_pub = nh.createTimer(ros::Duration(0.02), &UAV_estimator::timercb_pub_vision_pose, this);
    }
}
