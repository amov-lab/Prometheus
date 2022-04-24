#include "uav_estimator.h"

UAV_estimator::UAV_estimator(ros::NodeHandle &nh)
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

    // 根据设定的定位来源订阅不同的定位数据
    if (location_source == prometheus_msgs::UAVState::MOCAP)
    {
        // 【订阅】mocap估计位置
        mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node" + uav_name + "/pose", 1, &UAV_estimator::mocap_cb, this);
    }
    else if (location_source == prometheus_msgs::UAVState::T265)
    {
        // 【订阅】T265估计位置
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
    }
    else if (location_source == prometheus_msgs::UAVState::UWB)
    {
        // uwb todo
    }
    else
    {
        cout << YELLOW << node_name << ": wrong location_source param, no external location information input!" << TAIL << endl;
    }

    // 【发布】无人机状态合集,包括位置\速度\姿态\模式等,供上层节点使用
    uav_state_pub = nh.advertise<prometheus_msgs::UAVState>(uav_name + "/prometheus/state", 1);

    // 【发布】无人机位置和偏航角，传输至PX4_EKF2模块用于位置姿态估计 坐标系 ENU系
    px4_vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 1);

    // 【发布】无人机里程计,主要用于RVIZ显示
    uav_odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/prometheus/odom", 1);

    // 【发布】无人机运动轨迹,主要用于RVIZ显示
    uav_trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/prometheus/trajectory", 1);

    // 【发布】无人机位置(带图标),用于RVIZ显示
    uav_mesh_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/prometheus/uav_mesh", 1);

    // 【发布】运行状态信息(-> 通信节点 -> 地面站)
    ground_station_info_pub = nh.advertise<prometheus_msgs::TextInfo>("/uav" + std::to_string(uav_id) + "/prometheus/text_info", 1);

    if (location_source == prometheus_msgs::UAVState::MOCAP || location_source == prometheus_msgs::UAVState::T265 || location_source == prometheus_msgs::UAVState::GAZEBO)
    {
        // 【定时器】当需要使用外部定位设备时，需要定时发送vision信息至飞控,并保证一定频率
        // 此处是否可以检查PX4参数设置？
        timer_px4_vision_pub = nh.createTimer(ros::Duration(0.02), &UAV_estimator::timercb_pub_vision_pose, this);
    }

    // 【定时器】定时发布 uav_state, uav_odom 保证50Hz以上
    timer_uav_state_pub = nh.createTimer(ros::Duration(0.02), &UAV_estimator::timercb_pub_uav_state, this);

    // 【定时器】定时发布 rviz显示,保证1Hz以上
    timer_rviz_pub = nh.createTimer(ros::Duration(0.2), &UAV_estimator::timercb_rviz, this);

    // 变量初始化
    uav_state.uav_id = uav_id;
    uav_state.state = prometheus_msgs::UAVState::ready;
    uav_state.connected = false;
    uav_state.armed = false;
    uav_state.mode = "";
    uav_state.location_source = location_source;
    uav_state.odom_valid = false;
    uav_state.gps_status = prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_GPS;
    uav_state.position[0] = 0.0;
    uav_state.position[1] = 0.0;
    uav_state.position[2] = 0.0;
    //该经纬度设置为阿木实验室附近测试场地(小花园)的经纬度
    uav_state.latitude = 30.7852600; // 此处处置设置为阿木实验室经纬度，todo
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

    // 【函数】打印参数
    printf_param();
    cout << GREEN << node_name << " init! " << TAIL << endl;

    // 地面站消息打印
    text_info.header.stamp = ros::Time::now();
    text_info.MessageType = prometheus_msgs::TextInfo::INFO;
    text_info.Message = node_name + " init.";
    ground_station_info_pub.publish(text_info);
}

void UAV_estimator::px4_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_state.connected = msg->connected;
    uav_state.armed = msg->armed;
    uav_state.mode = msg->mode;

    uav_state_update = true;
}

void UAV_estimator::px4_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_state.position[0] = msg->pose.position.x;
    uav_state.position[1] = msg->pose.position.y;
    uav_state.position[2] = msg->pose.position.z;
}

void UAV_estimator::px4_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    uav_state.latitude = msg->latitude;
    uav_state.longitude = msg->longitude;
    uav_state.altitude = msg->altitude;
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

void UAV_estimator::gps_status_cb(const mavros_msgs::GPSRAW::ConstPtr &msg)
{
    uav_state.gps_status = msg->fix_type;
}

void UAV_estimator::mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    mocap_pose = *msg;
    get_mocap_stamp = ros::Time::now(); // 记录时间戳，防止超时
}

void UAV_estimator::gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    gazebo_pose.header = msg->header;
    gazebo_pose.pose = msg->pose.pose;
    get_gazebo_stamp = ros::Time::now(); // 记录时间戳，防止超时
}

void UAV_estimator::t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    t265_pose.header = msg->header;
    t265_pose.pose = msg->pose.pose;
    get_t265_stamp = ros::Time::now(); // 记录时间戳，防止超时
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
    // 检查odom状态
    odom_state = check_uav_odom();

    if (odom_state == 1 && last_odom_state != 1)
    {
        cout << RED << node_name << "--->  Odom invalid: Get Vision Pose Timeout! " << TAIL << endl;
    }
    else if (odom_state == 2 && last_odom_state != 2)
    {
        cout << RED << node_name << "--->  Odom invalid: Velocity too large! " << TAIL << endl;
    }
    else if (odom_state == 3 && last_odom_state != 3)
    {
        cout << RED << node_name << "--->  Odom invalid: vision_pose_error! " << TAIL << endl;
    }else if (odom_state == 4 && last_odom_state != 4)
    {
        cout << RED << node_name << "--->  Odom invalid: GPS/RTK location error! " << TAIL << endl;
    }else if (odom_state == 5 && last_odom_state != 5)
    {
        cout << YELLOW << node_name << "--->  Odom invalid: RTK not fixed! " << TAIL << endl;
    }

    if (odom_state == 9)
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

    // odom失效可能原因2：无人机合速度过大，认为定位模块失效
    Eigen::Vector2d uav_vel_xy = Eigen::Vector2d(uav_state.velocity[0], uav_state.velocity[1]);
    if (uav_vel_xy.norm() > maximum_safe_vel_xy || uav_state.velocity[2] > maximum_safe_vel_z)
    {
        return 2;
    }

    // odom失效可能原因3：无人机位置与外部定位设备原始值相差过多
    if (vision_pose_error)
    {
        return 3;
    }

    // GPS,RTK,UWB 这些需要做什么检查确认吗，todo
    // odom失效可能原因4:GPS定位模块数据异常,无法获取定位数据
    if(location_source == prometheus_msgs::UAVState::GPS)
    {
        if(uav_state.gps_status != prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX)
        {
            return 4;
        }
    }

    if(location_source == prometheus_msgs::UAVState::RTK)
    {
        if(uav_state.gps_status < prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX)
        {
            return 4;
        }
        // odom失效可能原因5:RTK定位精度较差(非odom失效状态)
        else if(uav_state.gps_status <= prometheus_msgs::UAVState::GPS_FIX_TYPE_RTK_FLOATR)
        {
            return 5;
        }
    }
    //UWB todo


    return 9;
}

void UAV_estimator::timercb_pub_uav_state(const ros::TimerEvent &e)
{
    if (!uav_state_update)
    {
        return;
    }

    // 1，检查odom状态
    // 还有其他需要检查的吗，和李博、张灵商量 todo
    check_uav_state();

    uav_state.header.stamp = ros::Time::now();
    uav_state_pub.publish(uav_state);

    // 发布无人机当前odometry(有些节点需要Odometry这个数据类型)
    uav_odom.header.stamp = ros::Time::now();
    uav_odom.header.frame_id = "world";
    uav_odom.child_frame_id = "base_link";

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
    else
    {
        return;
    }

    Eigen::Vector3d pos_vision = Eigen::Vector3d(vision_pose.pose.position.x, vision_pose.pose.position.y, vision_pose.pose.position.z);
    Eigen::Vector3d pos_px4 = Eigen::Vector3d(uav_state.position[0], uav_state.position[1], uav_state.position[2]);

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

    px4_vision_pose_pub.publish(vision_pose);
}

void UAV_estimator::timercb_rviz(const ros::TimerEvent &e)
{
    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped uav_pos;
    uav_pos.header.stamp = ros::Time::now();
    uav_pos.header.frame_id = "world";
    uav_pos.pose.position.x = uav_state.position[0];
    uav_pos.pose.position.y = uav_state.position[1];
    uav_pos.pose.position.z = uav_state.position[2];

    uav_pos.pose.orientation = uav_state.attitude_q;

    //发布无人机的位姿 和 轨迹 用作rviz中显示
    odom_vector.insert(odom_vector.begin(), uav_pos);
    if (odom_vector.size() > TRA_WINDOW)
    {
        odom_vector.pop_back();
    }

    nav_msgs::Path uav_trajectory;
    uav_trajectory.header.stamp = ros::Time::now();
    uav_trajectory.header.frame_id = "world";
    uav_trajectory.poses = odom_vector;
    uav_trajectory_pub.publish(uav_trajectory);

    visualization_msgs::Marker meshROS;
    // Mesh model
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

    // 发布TF用于RVIZ显示
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";       //相对于世界坐标系
    tfs.header.stamp = ros::Time::now(); //时间戳

    //  |----坐标系 ID
    tfs.child_frame_id = uav_name + "/lidar_link"; //子坐标系，无人机的坐标系

    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = uav_state.position[0];
    tfs.transform.translation.y = uav_state.position[1];
    tfs.transform.translation.z = uav_state.position[2];
    //  |--------- 四元数设置
    tfs.transform.rotation = uav_state.attitude_q;

    //  |--------- 广播器发布数据
    broadcaster.sendTransform(tfs);
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
        cout << GREEN << "PX4 State:  [ Connected ] ";
    }
    else
    {
        cout << RED << "PX4 State:[ Unconnected ] ";
    }
    //是否上锁
    if (uav_state.armed == true)
    {
        cout << GREEN << "[ Armed ] ";
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
    case prometheus_msgs::UAVState::UWB:
        cout << GREEN << "Location: [ UWB ] " << TAIL;
        // todo
        break;
    }

    if(uav_state.odom_valid)
    {
        cout << GREEN << "Odom State: [ Valid ] " << TAIL << endl;
    }
    else
    {
        cout << RED << "Odom State: [ Invalid ] " << TAIL << endl;
    }

    cout << GREEN << "UAV_pos [X Y Z] : " << uav_state.position[0] << " [ m ] " << uav_state.position[1] << " [ m ] " << uav_state.position[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "UAV_vel [X Y Z] : " << uav_state.velocity[0] << " [m/s] " << uav_state.velocity[1] << " [m/s] " << uav_state.velocity[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "UAV_att [R P Y] : " << uav_state.attitude[0] * 180 / M_PI << " [deg] " << uav_state.attitude[1] * 180 / M_PI << " [deg] " << uav_state.attitude[2] * 180 / M_PI << " [deg] " << TAIL << endl;
    
    cout << GREEN << "Battery Voltage : " << uav_state.battery_state << " [V] " << "  Battery Percent : " << uav_state.battery_percetage << TAIL << endl;
}

void UAV_estimator::printf_gps_status()
{
    // 确认一下，哪些是红色，哪些是绿色，todo...
    if(location_source == prometheus_msgs::UAVState::GPS)
    {
        switch (uav_state.gps_status)
        {
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_GPS:
            cout << RED  << " [GPS_FIX_TYPE_NO_GPS] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_FIX:
            cout << RED  << " [GPS_FIX_TYPE_NO_FIX] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_2D_FIX:
            cout << YELLOW  << " [GPS_FIX_TYPE_2D_FIX] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX:
            cout << GREEN  << " [GPS_FIX_TYPE_3D_FIX] " << TAIL << endl;
            break;
        }
    }
    if(location_source == prometheus_msgs::UAVState::RTK)
    {
        switch (uav_state.gps_status)
        {
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_GPS:
            cout << RED  << " [GPS_FIX_TYPE_NO_GPS] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_FIX:
            cout << RED  << " [GPS_FIX_TYPE_NO_FIX] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_2D_FIX:
            cout << RED  << " [GPS_FIX_TYPE_2D_FIX] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX:
            cout << YELLOW  << " [GPS_FIX_TYPE_3D_FIX] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_DGPS:
            cout << YELLOW  << " [GPS_FIX_TYPE_DGPS] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_RTK_FLOATR:
            cout << YELLOW  << " [GPS_FIX_TYPE_RTK_FLOATR] " << TAIL << endl;
            break;
        case prometheus_msgs::UAVState::GPS_FIX_TYPE_RTK_FIXEDR:
            cout << GREEN  << " [GPS_FIX_TYPE_RTK_FIXEDR] " << TAIL << endl;
            break;
        }
    }
    
    // 确定下单位，todo
    cout << GREEN << "GPS [lat lon alt] : " << uav_state.latitude << " [ deg ] " << uav_state.longitude << " [ deg ] " << uav_state.altitude << " [ m ] " << TAIL << endl;
}

void UAV_estimator::printf_param()
{
    cout << ">>>>>>>>>>>>>>>> UAV_estimator Param <<<<<<<<<<<<<<<<" << endl;
    cout << "uav_id : " << uav_id << " " << endl;
    cout << "maximum_safe_vel_xy           : " << maximum_safe_vel_xy << " [m/s] " << endl;
    cout << "maximum_safe_vel_z            : " << maximum_safe_vel_z << " [m/s] " << endl;
    cout << "maximum_vel_error_for_vision  : " << maximum_vel_error_for_vision << " [m/s] " << endl;

    if (location_source == prometheus_msgs::UAVState::GAZEBO)
    {
        cout << "location_source: [GAZEBO] " << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::MOCAP)
    {
        cout << "location_source: [MOCAP] " << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::T265)
    {
        cout << "location_source: [T265] " << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::FAKE_ODOM)
    {
        cout << "location_source: [FAKE_ODOM] " << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::GPS)
    {
        cout << "location_source: [GPS] " << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::RTK)
    {
        cout << "location_source: [GPS] " << endl;
    }
    else if (location_source == prometheus_msgs::UAVState::UWB)
    {
        cout << "location_source: [GPS] " << endl;
    }
    else
    {
        cout << "location_source: [UNKNOW] " << endl;
    }
}

// 怎么考虑无人机试飞的初始化问题？
// 连接上PX4是一个状态
// odom来源一切正常是一个状态
// 具备起飞条件是另一个状态
// 能否解锁（用户自行检查 还是我们代检查？）
// 打印不能解锁飞控的报错
// 打印不能呢个切入定点模式的报错
// 能够切入定点模式

//    //确认无人机是否能切入定点模式并解锁(遥控器是否会导致无人机起飞)
//     this->mavros_interface_.type = prometheus_msgs::MavrosInterface::SET_MODE;
//     this->mavros_interface_.mode = "POSCTL";
//     this->mavros_interface_pub_.publish(this->mavros_interface_);
//     bool loop_flag = true;
//     int count = 0;
//     while (loop_flag)
//     {
//         ros::spinOnce();
//         if (this->uav_state_.mode == "POSCTL")
//         {
//             loop_flag = false;
//         }
//         count++;
//         if (count >= setmode_timeout_ * 10)
//         {
//             loop_flag = false;
//             this->station_feedback_.MessageType = prometheus_msgs::StationFeedback::ERROR;
//             this->station_feedback_.Message = "UAV[" + std::to_string(this->agent_id_) + "] init failed, cannot set to [POSCTL] mode";
//             sendStationFeedback();
//             this->swarm_command_.Swarm_CMD = prometheus_msgs::SwarmCommand::Ready;
//             return false;
//         }
//         usleep(100000);
//     }

//     this->mavros_interface_.type = prometheus_msgs::MavrosInterface::ARMING;
//     this->mavros_interface_.arming = true;
//     this->mavros_interface_pub_.publish(this->mavros_interface_);
//     loop_flag = true;
//     //计数器
//     count = 0;

//     while (loop_flag)
//     {
//         ros::spinOnce();
//         if (this->uav_state_.armed)
//         {
//             loop_flag = false;
//             this->mavros_interface_.type = prometheus_msgs::MavrosInterface::ARMING;
//             this->mavros_interface_.arming = false;
//             this->mavros_interface_pub_.publish(this->mavros_interface_);
//             //是否考虑上锁指令发出后无法上锁的情况?
//         }
//         count++;
//         //以10hz来考虑,每秒计数器将增加10
//         if (count >= setmode_timeout_ * 10)
//         {
//             loop_flag = false;
//             this->station_feedback_.MessageType = prometheus_msgs::StationFeedback::ERROR;
//             this->station_feedback_.Message = "UAV[" + std::to_string(this->agent_id_) + "] init failed, cannot be armed";
//             sendStationFeedback();
//             this->swarm_command_.Swarm_CMD = prometheus_msgs::SwarmCommand::Ready;
//             return false;
//         }
//         usleep(100000);
//     }
