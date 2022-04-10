#include "uav_estimator.h"

UAV_estimator::UAV_estimator(ros::NodeHandle &nh)
{
    // 【参数】编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】定位源： 0 for mocap, 1 for t265, 2 for gazebo, 3 for fake_odom, 4 for GPS
    nh.param<int>("control/location_source", location_source, LOC_SOURCE::GPS);

    // 【变量】无人机名字
    uav_name = "/uav" + std::to_string(uav_id);
    // 【变量】节点名字
    node_name = "[uav_estimator_uav" + std::to_string(uav_id) + "]";

    // 【函数】打印参数
    printf_param();
    cout << GREEN << node_name << " init! "<< TAIL << endl;

    // 【订阅】无人机当前状态 - 来自飞控
    px4_state_sub = nh.subscribe<mavros_msgs::State>("/uav"+std::to_string(uav_id)+ "/mavros/state", 1, &UAV_estimator::px4_state_cb, this);

    // 【订阅】无人机电池状态 - 来自飞控
    px4_battery_sub = nh.subscribe<sensor_msgs::BatteryState>("/uav"+std::to_string(uav_id)+ "/mavros/battery", 1, &UAV_estimator::px4_battery_cb, this);

    // 【订阅】无人机当前位置 坐标系:ENU系 （此处注意，所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系）
    px4_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav"+std::to_string(uav_id)+ "/mavros/local_position/pose", 1, &UAV_estimator::px4_pos_cb, this);

    // 【订阅】无人机当前速度 坐标系:ENU系
    px4_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav"+std::to_string(uav_id)+ "/mavros/local_position/velocity_local", 1, &UAV_estimator::px4_vel_cb, this);

    // 【订阅】无人机当前经纬度
    px4_global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>("/uav"+std::to_string(uav_id)+ "/mavros/global_position/global", 1, &UAV_estimator::px4_global_pos_cb, this);

    // 【订阅】无人机当前欧拉角 坐标系:ENU系
    px4_attitude_sub = nh.subscribe<sensor_msgs::Imu>("/uav"+std::to_string(uav_id)+ "/mavros/imu/data", 1, &UAV_estimator::px4_att_cb, this); 

    if(location_source == LOC_SOURCE::MOCAP)
    {
        // 【订阅】mocap估计位置
        mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/uav"+std::to_string(uav_id)+ "/pose", 1, &UAV_estimator::mocap_cb, this);
    }else if(location_source == LOC_SOURCE::T265)
    {
        // 【订阅】T265估计位置
    }else if(location_source == LOC_SOURCE::GAZEBO)
    {
        // 【订阅】gazebo仿真真值
        gazebo_sub = nh.subscribe<nav_msgs::Odometry>("/uav"+std::to_string(uav_id)+ "/prometheus/ground_truth", 1, &UAV_estimator::gazebo_cb, this);
    }else if(location_source == LOC_SOURCE::FAKE_ODOM)
    {
        // 【订阅】fake odom
        fake_odom_sub = nh.subscribe<nav_msgs::Odometry>("/uav"+std::to_string(uav_id)+"/prometheus/fake_odom", 10, &UAV_estimator::fake_odom_cb, this);
    }else if(location_source == LOC_SOURCE::GPS)
    {
        // 【订阅】GPS状态，来自mavros
        gps_status_sub = nh.subscribe<mavros_msgs::GPSRAW>("/uav"+std::to_string(uav_id)+"/mavros/gpsstatus/gps1/raw", 10, &UAV_estimator::gps_status_cb, this);
    }else
    {
        cout << YELLOW << node_name << ": wrong location_source param, no external location information input!"<< TAIL << endl;
    }

    // 【发布】无人机状态合集,包括位置\速度\姿态\模式等,供上层节点使用
    uav_state_pub = nh.advertise<prometheus_msgs::UAVState>("/uav"+std::to_string(uav_id)+ "/prometheus/state", 1);

    // 【发布】无人机位置和偏航角，传输至PX4_EKF2模块用于位置姿态估计 坐标系 ENU系 
    px4_vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav"+std::to_string(uav_id)+ "/mavros/vision_pose/pose", 1);

    // 【发布】无人机里程计,主要用于RVIZ显示
    uav_odom_pub = nh.advertise<nav_msgs::Odometry>("/uav"+std::to_string(uav_id)+ "/prometheus/odom", 1);

    // 【发布】无人机运动轨迹,主要用于RVIZ显示
    uav_trajectory_pub = nh.advertise<nav_msgs::Path>("/uav"+std::to_string(uav_id)+ "/prometheus/trajectory", 1);

    // 【发布】无人机位置(带图标),用于RVIZ显示
    uav_mesh_pub = nh.advertise<visualization_msgs::Marker>("/uav"+std::to_string(uav_id)+ "/prometheus/uav_mesh", 1);

    if(location_source == LOC_SOURCE::MOCAP || location_source == LOC_SOURCE::T265 || location_source == LOC_SOURCE::GAZEBO)
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
    uav_state.location_source = location_source;
    uav_state.mode = "";
    uav_state.odom_valid = false;
    uav_state.position[0] = 0.0;
    uav_state.position[1] = 0.0;
    uav_state.position[2] = 0.0;
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
}

void UAV_estimator::px4_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_state.connected = msg->connected;
    uav_state.armed = msg->armed;
    uav_state.mode = msg->mode;

    uav_state_update = true;
}

void UAV_estimator::px4_battery_cb(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    uav_state.battery_state = msg->voltage;
    uav_state.battery_percetage = msg->percentage;
}

void UAV_estimator::px4_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_state.position[0] = msg->pose.position.x;
    uav_state.position[1] = msg->pose.position.y;
    uav_state.position[2] = msg->pose.position.z;
}

void UAV_estimator::px4_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    uav_state.global_position[0] = msg->latitude;
    uav_state.global_position[1] = msg->longitude;
    uav_state.global_position[2] = msg->altitude;
}

void UAV_estimator::px4_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    uav_state.velocity[0] = msg->twist.linear.x;
    uav_state.velocity[1] = msg->twist.linear.y;
    uav_state.velocity[2] = msg->twist.linear.z;
}

void UAV_estimator::px4_att_cb(const sensor_msgs::Imu::ConstPtr& msg)
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

void UAV_estimator::gps_status_cb(const mavros_msgs::GPSRAW::ConstPtr& msg)
{
    uav_state.gps_status = msg->fix_type;
}

void UAV_estimator::mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(location_source != LOC_SOURCE::MOCAP)
    {
        return;
    }

    mocap_pose = *msg;
    get_mocap_stamp = ros::Time::now();
}

void UAV_estimator::gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(location_source != LOC_SOURCE::GAZEBO)
    {
        return;
    }
    
    gazebo_pose.header = msg->header;
    gazebo_pose.pose = msg->pose.pose;
    get_gazebo_stamp = ros::Time::now();
}

void UAV_estimator::fake_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(location_source != LOC_SOURCE::FAKE_ODOM)
    {
        return;
    }

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
    uav_state.attitude_q  = msg->pose.pose.orientation;
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

void UAV_estimator::t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(location_source != LOC_SOURCE::T265)
    {
        return;
    }

    t265_pose.header = msg->header;
    t265_pose.pose = msg->pose.pose;
    get_t265_stamp = ros::Time::now();
}

void UAV_estimator::timercb_pub_vision_pose(const ros::TimerEvent &e)
{
    if(location_source == LOC_SOURCE::FAKE_ODOM || location_source == LOC_SOURCE::GPS)
    {
        return;
    }

    if(location_source == LOC_SOURCE::GAZEBO)
    {
        vision_pose = gazebo_pose;
    }else if(location_source == LOC_SOURCE::MOCAP)
    {
        vision_pose = mocap_pose;
    }else if(location_source == LOC_SOURCE::T265)
    {
        vision_pose = t265_pose;
    }else
    {
        return;
    }

    Eigen::Vector3d pos_vision = Eigen::Vector3d(vision_pose.pose.position.x,vision_pose.pose.position.y,vision_pose.pose.position.z);
    Eigen::Vector3d pos_px4    = Eigen::Vector3d(uav_state.position[0],uav_state.position[1],uav_state.position[2]);
    
    // vision位置和px4回传位置相差较多，一般是PX4中EKF2参数设置错误导致PX4没有收到vision定位数据导致
    if( (pos_vision-pos_px4).norm()> 0.5)
    {
        vision_pose_error = true;
    }else
    {
        vision_pose_error = false;
    }
    

    px4_vision_pose_pub.publish(vision_pose);
}

/**
 * @brief 检查无人机状态，包括无人机与PX4连接状态、是否有本地位置数据
 * 是否解锁状态？是否处于offboard模式状态？在此处有点鸡肋
 * 
 * @return int 
 */
int UAV_estimator::check_uav_state()
{
    ros::Time time_now = ros::Time::now();

    if(!uav_state.connected)
    {
        cout << RED << node_name << "--->  PX4 Unconnected ! " << TAIL <<endl; 
        return 0;
    }

    // if(!uav_state.armed)
    // {
    //     cout << YELLOW << node_name << "--->  PX4 Armed ! " << TAIL <<endl; 
    //     return 0;
    // }

    // if(uav_state.mode != "OFFBOARD")
    // {
    //     cout << YELLOW << node_name << "--->  PX4 not in OFFBOARD mode ! " << TAIL <<endl; 
    //     return 0;
    // }

    return 9;
}

int UAV_estimator::check_uav_odom()
{
    ros::Time time_now = ros::Time::now();

    // odom失效可能原因1：外部定位数据接收超时
    // odom失效可能原因2：无人机合速度大于3米每秒，认为定位模块失效
    // odom失效可能原因3：无人机位置与外部定位设备原始值相差过多
    if(location_source == LOC_SOURCE::GAZEBO && (time_now - get_gazebo_stamp).toSec() > GAZEBO_TIMEOUT)
    {
        return 1;
    }else if(location_source == LOC_SOURCE::MOCAP && (time_now - get_mocap_stamp).toSec() > MOCAP_TIMEOUT)
    {
        return 1;
    }else if(location_source == LOC_SOURCE::T265 && (time_now - get_t265_stamp).toSec() > T265_TIMEOUT)
    {
        return 1;
    }

    // 速度过大
    // 2022.3.9 速度范围先设置较大，在指点飞行过程中速度过大导致odom为false，进入失控保护
    Eigen::Vector3d uav_vel = Eigen::Vector3d(uav_state.velocity[0],uav_state.velocity[1],uav_state.velocity[2]); 
    if(uav_vel.norm() > 5.0 || uav_vel(2) > 4.0)
    {
        return 2;
    }

    if(vision_pose_error)
    {
        return 3;
    }

    return 9;
}

void UAV_estimator::timercb_pub_uav_state(const ros::TimerEvent &e)
{
    if(!uav_state_update)
    {
        return;
    }

    static int printf_count = 0;
    printf_count++;

    int odom_state = check_uav_odom();

    if(printf_count == 1)
    {
        // 检查无人机状态: 连接状态、上锁状态、模式状态
        check_uav_state();
        if(odom_state == 1)
        {
            cout << RED << node_name << "--->  Odom invalid: Get Vision Pose Timeout! " << TAIL <<endl; 
        }else if(odom_state == 2)
        {
            cout << RED << node_name << "--->  Odom invalid: Velocity too large! " << TAIL <<endl; 
        }else if(odom_state == 3)
        {
            cout << RED << node_name << "--->  Odom invalid: vision_pose_error! " << TAIL <<endl; 
        }
    }else if(printf_count > 100)
    {
        printf_count = 0;
    }

    if(odom_state == 9)
    {
        uav_state.odom_valid = true;
    }else
    {
        uav_state.odom_valid = false;
    }

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
    tfs.header.frame_id = "world";  //相对于世界坐标系
    tfs.header.stamp = ros::Time::now();  //时间戳
    
    //  |----坐标系 ID
    tfs.child_frame_id = uav_name + "/lidar_link";  //子坐标系，无人机的坐标系

    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = uav_state.position[0];
    tfs.transform.translation.y = uav_state.position[1];
    tfs.transform.translation.z = uav_state.position[2];
    //  |--------- 四元数设置  
    tfs.transform.rotation = uav_state.attitude_q;

    //  |--------- 广播器发布数据
    broadcaster.sendTransform(tfs);
}

void UAV_estimator::printf_param()
{
    cout <<">>>>>>>>>>>>>>>> UAV_estimator Param <<<<<<<<<<<<<<<<" <<endl;

    if(location_source == LOC_SOURCE::GAZEBO)
    {
        cout << "location_source: [GAZEBO] "<<endl;
    }else if(location_source == LOC_SOURCE::MOCAP)
    {
        cout << "location_source: [MOCAP] "<<endl;
    }else if(location_source == LOC_SOURCE::T265)
    {
        cout << "location_source: [T265] "<<endl;
    }else if(location_source == LOC_SOURCE::FAKE_ODOM)
    {
        cout << "location_source: [FAKE_ODOM] "<<endl;
    }else if(location_source == LOC_SOURCE::GPS)
    {
        cout << "location_source: [GPS] "<<endl;
    }else
    {
        cout << "location_source: [UNKNOW] "<<endl;
    }
}

