#include "drl_actuator.h"
namespace drl_ns
{
void drl_actuator::init(ros::NodeHandle& nh, int id, Eigen::Vector3d _init_pos, double _init_yaw)
{
    agent_id = id;
    model_name = "fake_ugv_" + std::to_string(agent_id);
    node_name = "fake_ugv" + std::to_string(agent_id);
    get_move_cmd = false;
    ugv_state_update = false;
    cmd_id = 0;
    block_size = 0.2;
    dt = 0.1;
    car_model = 0;      // 0代表全向轮，1代表差速轮

    // 初始化 ugv_state
    ugv_state.pos = _init_pos;
    ugv_state.vel << 0.0,0.0,0.0;
    ugv_state.euler << 0.0,0.0,_init_yaw;
    ugv_state.quat2 = quaternion_from_rpy(ugv_state.euler);
    ugv_state.quat.x = ugv_state.quat2.x();
    ugv_state.quat.y = ugv_state.quat2.y();
    ugv_state.quat.z = ugv_state.quat2.z();
    ugv_state.quat.w = ugv_state.quat2.w();
    // 初始化 gazebo_model_state
    gazebo_model_state.model_name = model_name;
    gazebo_model_state.pose.position.x = _init_pos[0];
    gazebo_model_state.pose.position.y = _init_pos[1];
    gazebo_model_state.pose.position.z = _init_pos[2];
    gazebo_model_state.pose.orientation.x = ugv_state.quat2.x();
    gazebo_model_state.pose.orientation.y = ugv_state.quat2.y();
    gazebo_model_state.pose.orientation.z = ugv_state.quat2.z();
    gazebo_model_state.pose.orientation.w = ugv_state.quat2.w();
    gazebo_model_state.reference_frame = "ground_plane::link";

    // 【订阅】 drl 离散动作   
    move_cmd_sub  = nh.subscribe<prometheus_drl::ugv_move_cmd>("/ugv"+std::to_string(agent_id) + "/move_cmd", 1, &drl_actuator::move_cmd_cb, this);
    // 【订阅】 drl 连续动作 
    vel_cmd_sub   = nh.subscribe<geometry_msgs::Twist>("/ugv"+std::to_string(agent_id) + "/cmd_vel", 1, &drl_actuator::vel_cmd_cb, this);
    // 【发布】 odom，用于RVIZ显示 
    fake_odom_pub = nh.advertise<nav_msgs::Odometry>("/ugv"+std::to_string(agent_id) + "/fake_odom", 1);
    // 【发布】mesh，用于RVIZ显示
    ugv_mesh_pub  = nh.advertise<visualization_msgs::Marker>("/ugv"+std::to_string(agent_id) + "/ugv_mesh", 1);
    // 【定时器】
    fake_odom_pub_timer = nh.createTimer(ros::Duration(0.05), &drl_actuator::fake_odom_pub_cb, this);

    cout << GREEN  << node_name << "---> drl_actuator init sucess in position: " << _init_pos[0] <<" [ m ] "<<_init_pos[1] <<" [ m ] "<< TAIL <<endl;
}

void drl_actuator::reset(Eigen::Vector3d _init_pos, double _init_yaw)
{
    // 初始化 ugv_state
    ugv_state.pos = _init_pos;
    ugv_state.vel << 0.0,0.0,0.0;
    ugv_state.euler << 0.0,0.0,_init_yaw;
    ugv_state.quat2 = quaternion_from_rpy(ugv_state.euler);
    ugv_state.quat.x = ugv_state.quat2.x();
    ugv_state.quat.y = ugv_state.quat2.y();
    ugv_state.quat.z = ugv_state.quat2.z();
    ugv_state.quat.w = ugv_state.quat2.w();

    cmd_id = 0;
    action.ID = 0;
    action.CMD = prometheus_drl::ugv_move_cmd::HOLD;
}

void drl_actuator::move_cmd_cb(const prometheus_drl::ugv_move_cmd::ConstPtr& msg)
{
    action = *msg;

    if(action.ID <= cmd_id)
    {
        cout << RED << node_name << "---> wrong cmd id."<< TAIL << endl;
        return;
    }
    cmd_id = action.ID;
    get_move_cmd = true;

    if(action.CMD == prometheus_drl::ugv_move_cmd::HOLD)
    {

    }else if(action.CMD == prometheus_drl::ugv_move_cmd::FORWARD)
    {
        ugv_state.pos[0] = ugv_state.pos[0] + block_size;
    }else if(action.CMD == prometheus_drl::ugv_move_cmd::BACK)
    {
        ugv_state.pos[0] = ugv_state.pos[0] - block_size;
    }else if(action.CMD == prometheus_drl::ugv_move_cmd::LEFT)
    {
        ugv_state.pos[1] = ugv_state.pos[1] + block_size;
    }else if(action.CMD == prometheus_drl::ugv_move_cmd::RIGHT)
    {
        ugv_state.pos[1] = ugv_state.pos[1] - block_size;
    }else
    {
        cout << RED << node_name << "---> wrong move cmd."<< TAIL << endl;
    }
}

void drl_actuator::vel_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    if(car_model == 0)
    {
        // 默认vel_cmd_cb的回调频率是10Hz
        // 全向小车
        ugv_state.euler << 0.0,0.0,0.0;
        ugv_state.quat2 = quaternion_from_rpy(ugv_state.euler);
        ugv_state.quat.x = ugv_state.quat2.x();
        ugv_state.quat.y = ugv_state.quat2.y();
        ugv_state.quat.z = ugv_state.quat2.z();
        ugv_state.quat.w = ugv_state.quat2.w();

        ugv_state.vel[0] = msg->linear.x;
        ugv_state.vel[1] = msg->linear.y;
        ugv_state.vel[2] = 0.0;

        ugv_state.pos = ugv_state.pos + ugv_state.vel * dt;
    }else if(car_model == 0)
    {
        // 默认vel_cmd_cb的回调频率是10Hz
        // 差速模型小车
        ugv_state.euler[0] = 0.0;
        ugv_state.euler[1] = 0.0;
        ugv_state.euler[2] = ugv_state.euler[2] + msg->angular.z * dt;

        ugv_state.quat2 = quaternion_from_rpy(ugv_state.euler);
        ugv_state.quat.x = ugv_state.quat2.x();
        ugv_state.quat.y = ugv_state.quat2.y();
        ugv_state.quat.z = ugv_state.quat2.z();
        ugv_state.quat.w = ugv_state.quat2.w();

        ugv_state.vel_body[0] = msg->linear.x;
        ugv_state.vel_body[1] = 0.0;
        ugv_state.vel_body[2] = 0.0;

        ugv_state.vel[0] = ugv_state.vel_body[0] * cos(ugv_state.euler[2]);
        ugv_state.vel[1] = ugv_state.vel_body[0] * sin(ugv_state.euler[2]);
        ugv_state.vel[2] = 0.0;

        ugv_state.pos = ugv_state.pos + ugv_state.vel * dt;
    }

}

void drl_actuator::fake_odom_pub_cb(const ros::TimerEvent &e)
{
    // 发布fake odom
    fake_odom.header.stamp = ros::Time::now();
    fake_odom.header.frame_id = "world";
    fake_odom.child_frame_id = "base_link";
    fake_odom.pose.pose.position.x = ugv_state.pos[0];
    fake_odom.pose.pose.position.y = ugv_state.pos[1];
    fake_odom.pose.pose.position.z = ugv_state.pos[2];
    fake_odom.pose.pose.orientation = ugv_state.quat;
    fake_odom.twist.twist.linear.x = ugv_state.vel[0];
    fake_odom.twist.twist.linear.y = ugv_state.vel[1];
    fake_odom.twist.twist.linear.z = ugv_state.vel[2];
    fake_odom_pub.publish(fake_odom);

    // 发布mesh
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = "world";
    meshROS.header.stamp = ros::Time::now();
    meshROS.ns = "ugv_mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = ugv_state.pos[0];
    meshROS.pose.position.y = ugv_state.pos[1];
    meshROS.pose.position.z = ugv_state.pos[2];
    meshROS.pose.orientation = ugv_state.quat;
    meshROS.scale.x = 0.8/4.5;
    meshROS.scale.y = 0.8/4.5;
    meshROS.scale.z = 0.8/4.5;
    meshROS.color.a = 1.0;
    meshROS.color.r = 0.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 0.5;
    meshROS.mesh_resource = std::string("package://prometheus_drl/meshes/car.dae");
    ugv_mesh_pub.publish(meshROS); 


    // 发布TF用于RVIZ显示
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";  //相对于世界坐标系
    tfs.header.stamp = ros::Time::now();  //时间戳
    
    //  |----坐标系 ID
    tfs.child_frame_id =  "ugv" + std::to_string(agent_id) + "/lidar_link";  //子坐标系，无人机的坐标系

    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = ugv_state.pos[0];
    tfs.transform.translation.y = ugv_state.pos[1];
    tfs.transform.translation.z = ugv_state.pos[2];
    //  |--------- 四元数设置  
    tfs.transform.rotation.x = fake_odom.pose.pose.orientation.x;
    tfs.transform.rotation.y = fake_odom.pose.pose.orientation.y;
    tfs.transform.rotation.z = fake_odom.pose.pose.orientation.z;
    tfs.transform.rotation.w = fake_odom.pose.pose.orientation.w;

    //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);

    // 更新 gazebo_model_state
    // 注意：这个话题发布的是相对位置，且无法移除初始位置的影响（因此将所有无人机初始位置设为0）
    // 注意：他这个坐标转换是先转角度再加位移
    gazebo_model_state.model_name = model_name;
    gazebo_model_state.pose.position.x = ugv_state.pos[0];
    gazebo_model_state.pose.position.y = ugv_state.pos[1];
    gazebo_model_state.pose.position.z = ugv_state.pos[2];
    gazebo_model_state.pose.orientation = ugv_state.quat;
    gazebo_model_state.reference_frame = "ground_plane::link";
}

void drl_actuator::printf_cb()
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    if(action.CMD == prometheus_drl::ugv_move_cmd::HOLD)
    {
        cout << GREEN  << "Action : [" << action.ID << "] " << "[  HOLD   ]" << TAIL <<endl;
    }else if(action.CMD == prometheus_drl::ugv_move_cmd::FORWARD)
    {
        cout << GREEN  << "Action : [" << action.ID << "] " << "[ FORWARD ]" << TAIL <<endl;
    }else if(action.CMD == prometheus_drl::ugv_move_cmd::BACK)
    {
        cout << GREEN  << "Action : [" << action.ID << "] " << "[  BACK   ]" << TAIL <<endl;
    }else if(action.CMD == prometheus_drl::ugv_move_cmd::LEFT)
    {
        cout << GREEN  << "Action : [" << action.ID << "] " << "[  LEFT   ]" << TAIL <<endl;
    }else if(action.CMD == prometheus_drl::ugv_move_cmd::RIGHT)
    {
        cout << GREEN  << "Action : [" << action.ID << "] " << "[  RIGHT  ]" << TAIL <<endl;
    }

}

Eigen::Vector3d drl_actuator::get_ugv_pos()
{
    return ugv_state.pos;
}

gazebo_msgs::ModelState drl_actuator::get_model_state()
{
    return gazebo_model_state;
}

// 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
Eigen::Quaterniond drl_actuator::quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
        // YPR - ZYX
        return Eigen::Quaterniond(
                        Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
                        );
}
}