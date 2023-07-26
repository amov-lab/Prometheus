#include "ugv_estimator.h"


UGV_estimator::UGV_estimator(ros::NodeHandle& nh)
{
     // 读取参数
    nh.param("ugv_id", this->ugv_id, 0);
    nh.param("sim_mode", this->sim_mode, false);
    nh.param("mesh_resource", this->mesh_resource, std::string("package://prometheus_ugv_control/meshes/car.dae"));


    this->ugv_name = "/ugv" + std::to_string(this->ugv_id);

     if(this->sim_mode)
    {
        // 【订阅】gazebo仿真真值
        this->gazebo_odom_sub = nh.subscribe<nav_msgs::Odometry>(this->ugv_name + "/prometheus/fake_odom", 1, &UGV_estimator::gazebo_cb, this);
    }else
    {
        // 【订阅】mocap估计位置
        this->mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ this->ugv_name + "/pose", 1, &UGV_estimator::mocap_pos_cb, this);

        // 【订阅】mocap估计速度
        // mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node"+ ugv_name + "/twist", 1, mocap_vel_cb);
    
        // 【订阅】电池状态(无人车底板电压)
        this->battery_sub = nh.subscribe<std_msgs::Float32>(this->ugv_name + "/PowerVoltage", 1, &UGV_estimator::battery_cb, this);
    }

    // 【发布】无人车状态合集,包括位置\速度\姿态\模式等,供上层节点使用
    this->ugv_state_pub = nh.advertise<prometheus_msgs::UgvState>(this->ugv_name + "/prometheus/ugv_state", 1);

    // 【发布】无人车里程计
    this->ugv_odom_pub = nh.advertise<nav_msgs::Odometry>(this->ugv_name + "/prometheus/ugv_odom", 1);

    // 【发布】无人车运动轨迹
    this->trajectory_pub = nh.advertise<nav_msgs::Path>(this->ugv_name + "/prometheus/ugv_trajectory", 1);

    // 【发布】mesh，用于RVIZ显示
    this->ugv_mesh_pub =  nh.advertise<visualization_msgs::Marker>(this->ugv_name + "/prometheus/ugv_mesh", 1);

    // 【定时器】发布ugv_state话题，50Hz
    ros::Timer timer_ugv_state_pub = nh.createTimer(ros::Duration(0.02), &UGV_estimator::timercb_ugv_state, this);

    // 【定时器】发布RVIZ显示相关话题，5Hz
    ros::Timer timer_rviz_pub = nh.createTimer(ros::Duration(0.2), &UGV_estimator::timercb_rviz, this);

    
    // 变量初始化
    this->ugv_state.battery = 0.0;
    this->ugv_state.position[0] = 0.0;
    this->ugv_state.position[1] = 0.0;
    this->ugv_state.position[2] = 0.0;
    this->ugv_state.velocity[0] = 0.0;
    this->ugv_state.velocity[1] = 0.0;
    this->ugv_state.velocity[2] = 0.0;
    this->ugv_state.attitude[0] = 0.0;
    this->ugv_state.attitude[1] = 0.0;
    this->ugv_state.attitude[2] = 0.0;
    this->ugv_state.attitude_q.x = 0.0;
    this->ugv_state.attitude_q.y = 0.0;
    this->ugv_state.attitude_q.z = 0.0;
    this->ugv_state.attitude_q.w = 1.0;

    //之前时刻的保存参数
    this->last_position_x = 0.0;
    this->last_position_y = 0.0;
    this->last_position_z = 0.0;
    this->last_time = get_time_in_sec(ros::Time::now());

    this->last_mocap_timestamp = ros::Time::now();
    this->mocap_first_time = true;

    cout << GREEN << "ugv_estimator_ugv_" <<  this->ugv_id << " init."<< TAIL <<endl; 

}


void UGV_estimator::mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // ugv_state赋值 - 位置
    this->ugv_state.position[0] = msg->pose.position.x;
    this->ugv_state.position[1] = msg->pose.position.y;
    this->ugv_state.position[2] = msg->pose.position.z;

    //计算速度
    // now_time = get_time_in_sec(ros::Time::now());
    // dt = now_time - last_time;
    this->dt  = 0.01;
    this->ugv_state.velocity[0] = (ugv_state.position[0] - last_position_x) / dt;
    this->ugv_state.velocity[1] = (ugv_state.position[1] - last_position_y) / dt;
    this->ugv_state.velocity[2] = (ugv_state.position[2] - last_position_z) / dt;

    //保存当前信息为上一时刻
    this->last_position_x = this->ugv_state.position[0];
    this->last_position_y = this->ugv_state.position[1];
    this->last_position_z = this->ugv_state.position[2];
    // last_time = now_time;

    // ugv_state赋值 - 四元数
    this->ugv_state.attitude_q = msg->pose.orientation;
    // ugv_state赋值 - 欧拉角
    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d euler_mocap = quaternion_to_euler(q_mocap);
    this->ugv_state.attitude[0] = euler_mocap[0];
    this->ugv_state.attitude[1] = euler_mocap[1];
    this->ugv_state.attitude[2] = euler_mocap[2];

    // 记录收到mocap的时间戳
    this->last_mocap_timestamp = ros::Time::now();
}

void UGV_estimator::mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // ugv_state赋值 - 速度
    this->ugv_state.velocity[0] = msg->twist.linear.x;
    this->ugv_state.velocity[1] = msg->twist.linear.y;
    this->ugv_state.velocity[2] = msg->twist.linear.z;
}

void UGV_estimator::battery_cb(const std_msgs::Float32::ConstPtr &msg)
{
    this->ugv_state.battery = msg->data;
}

void UGV_estimator::gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ugv_state赋值 - 位置
    this->ugv_state.position[0] = msg->pose.pose.position.x;
    this->ugv_state.position[1] = msg->pose.pose.position.y;
    this->ugv_state.position[2] = msg->pose.pose.position.z;
    // ugv_state赋值 - 速度
    this->ugv_state.velocity[0] = msg->twist.twist.linear.x;
    this->ugv_state.velocity[1] = msg->twist.twist.linear.y;
    this->ugv_state.velocity[2] = msg->twist.twist.linear.z;
    // ugv_state赋值 - 四元数
    this->ugv_state.attitude_q = msg->pose.pose.orientation;
    // ugv_state赋值 - 欧拉角
    Eigen::Quaterniond q_gazebo = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d euler_gazebo = quaternion_to_euler(q_gazebo);
    this->ugv_state.attitude[0] = euler_gazebo[0];
    this->ugv_state.attitude[1] = euler_gazebo[1];
    this->ugv_state.attitude[2] = euler_gazebo[2];
}

// 【获取当前时间函数】 单位：秒
float UGV_estimator::get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void UGV_estimator::timercb_ugv_state(const ros::TimerEvent &e)
{
    // 如果长时间未收到mocap数据，则一直给飞控发送旧数据，此处显示timeout
    if(!this->sim_mode && get_time_in_sec(this->last_mocap_timestamp) > TIMEOUT_MAX)
    {
        if(this->mocap_first_time)
        {
            this->mocap_first_time = false;
            return;
        }
        cout << RED << "Mocap Timeout : " <<  get_time_in_sec(this->last_mocap_timestamp)  << " [ s ]"<< TAIL <<endl; 
    }

    // 发布无人车状态
    this->ugv_state.header.stamp = ros::Time::now();
    this->ugv_state_pub.publish(this->ugv_state);

    // 发布无人车当前odometry
    this->ugv_odom.header.stamp = ros::Time::now();
    this->ugv_odom.header.frame_id = "world";
    this->ugv_odom.child_frame_id = "base_link";
    this->ugv_odom.pose.pose.position.x = this->ugv_state.position[0];
    this->ugv_odom.pose.pose.position.y = this->ugv_state.position[1];
    this->ugv_odom.pose.pose.position.z = this->ugv_state.position[2];
    this->ugv_odom.pose.pose.orientation = this->ugv_state.attitude_q;
    this->ugv_odom.twist.twist.linear.x = this->ugv_state.velocity[0];
    this->ugv_odom.twist.twist.linear.y = this->ugv_state.velocity[1];
    this->ugv_odom.twist.twist.linear.z = this->ugv_state.velocity[2];
    this->ugv_odom_pub.publish(ugv_odom);
}

void UGV_estimator::timercb_rviz(const ros::TimerEvent &e)
{
    // 发布无人车运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped ugv_pos;
    ugv_pos.header.stamp = ros::Time::now();
    ugv_pos.header.frame_id = "world";
    ugv_pos.pose.position.x = this->ugv_state.position[0];
    ugv_pos.pose.position.y = this->ugv_state.position[1];
    ugv_pos.pose.position.z = this->ugv_state.position[2];

    ugv_pos.pose.orientation = this->ugv_state.attitude_q;

    //发布无人车的位姿 和 轨迹 用作rviz中显示
    this->posehistory_vector_.insert(this->posehistory_vector_.begin(), ugv_pos);
    if (this->posehistory_vector_.size() > TRA_WINDOW)
    {
        this->posehistory_vector_.pop_back();
    }

    nav_msgs::Path ugv_trajectory;
    ugv_trajectory.header.stamp = ros::Time::now();
    ugv_trajectory.header.frame_id = "world";
    ugv_trajectory.poses = this->posehistory_vector_;
    this->trajectory_pub.publish(ugv_trajectory);

    // 发布mesh
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = "world";
    meshROS.header.stamp = ros::Time::now();
    meshROS.ns = "ugv_mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = this->ugv_state.position[0];
    meshROS.pose.position.y = this->ugv_state.position[1];
    meshROS.pose.position.z = this->ugv_state.position[2];
    meshROS.pose.orientation = this->ugv_state.attitude_q;
    meshROS.scale.x = 0.6/4.5;
    meshROS.scale.y = 0.6/4.5;
    meshROS.scale.z = 0.6/4.5;
    meshROS.color.a = 1.0;
    meshROS.color.r = 0.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 1.0;
    meshROS.mesh_resource = mesh_resource;
    this->ugv_mesh_pub.publish(meshROS); 

    // 发布TF用于RVIZ显示（激光雷达与无人车的tf）
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";  //相对于世界坐标系
    tfs.header.stamp = ros::Time::now();  //时间戳
    //  |----坐标系 ID
    tfs.child_frame_id = ugv_name + "/lidar_link";  //子坐标系，无人车的坐标系
    //  |----坐标系相对信息设置  偏移量  无人车相对于世界坐标系的坐标
    tfs.transform.translation.x = this->ugv_state.position[0];
    tfs.transform.translation.y = this->ugv_state.position[1];
    tfs.transform.translation.z = this->ugv_state.position[2];
    //  |--------- 四元数设置  
    tfs.transform.rotation = this->ugv_state.attitude_q;
    //  发布数据
    broadcaster.sendTransform(tfs);
}