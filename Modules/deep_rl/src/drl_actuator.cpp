#include "drl_actuator.h"
namespace drl_ns
{
void drl_actuator::init(ros::NodeHandle& nh, int id, Eigen::Vector3d _init_pos, double _init_yaw)
{
    // 模型前缀 - 默认为 ugv，无人机则设置为 uav
    nh.param<string>("agent_prefix", agent_prefix, "/ugv");
    // 动作模式 - 0 代表离散控制，1代表连续控制
    nh.param("action_mode", action_mode, 0);

    agent_id = id;
    if(agent_prefix == "/uav")
    {
        model_name = "fake_uav_" + std::to_string(agent_id);
        node_name = "fake_uav" + std::to_string(agent_id);
        actuator_model = 0;     
    }else if(agent_prefix == "/ugv")
    {
        model_name = "fake_ugv_" + std::to_string(agent_id);
        node_name = "fake_ugv" + std::to_string(agent_id);
        // actuator_model = 1;     // 差速
        actuator_model = 0;  // 全向
    }

    agent_name = agent_prefix + std::to_string(agent_id);
    get_move_cmd = false;
    cmd_id = 0;
    block_size = 0.2;
    dt = 0.1;

    // 初始化 agent_state
    agent_state.pos = _init_pos;
    agent_state.vel << 0.0,0.0,0.0;
    agent_state.euler << 0.0,0.0,_init_yaw;
    agent_state.quat2 = quaternion_from_rpy(agent_state.euler);
    agent_state.quat.x = agent_state.quat2.x();
    agent_state.quat.y = agent_state.quat2.y();
    agent_state.quat.z = agent_state.quat2.z();
    agent_state.quat.w = agent_state.quat2.w();
    // 初始化 gazebo_model_state
    gazebo_model_state.model_name = model_name;
    gazebo_model_state.pose.position.x = _init_pos[0];
    gazebo_model_state.pose.position.y = _init_pos[1];
    gazebo_model_state.pose.position.z = _init_pos[2];
    gazebo_model_state.pose.orientation.x = agent_state.quat2.x();
    gazebo_model_state.pose.orientation.y = agent_state.quat2.y();
    gazebo_model_state.pose.orientation.z = agent_state.quat2.z();
    gazebo_model_state.pose.orientation.w = agent_state.quat2.w();
    gazebo_model_state.reference_frame = "ground_plane::link";

    if(action_mode == 0)
    {
        // 【订阅】 drl 离散动作   
        move_cmd_sub = nh.subscribe<prometheus_drl::move_cmd>(agent_name + "/move_cmd", 1, &drl_actuator::move_cmd_cb, this);
    }else if(action_mode == 1)
    {
        // 【订阅】 drl 连续动作 
        vel_cmd_sub  = nh.subscribe<geometry_msgs::Twist>(agent_name + "/cmd_vel", 1, &drl_actuator::vel_cmd_cb, this);
    }
    // 【发布】 odom，用于RVIZ显示 
    fake_odom_pub = nh.advertise<nav_msgs::Odometry>(agent_name + "/fake_odom", 1);
    // 【发布】mesh，用于RVIZ显示
    mesh_pub  = nh.advertise<visualization_msgs::Marker>(agent_name + "/mesh", 1);
    // 【定时器】
    fake_odom_pub_timer = nh.createTimer(ros::Duration(0.05), &drl_actuator::fake_odom_pub_cb, this);

    cout << GREEN  << node_name << "---> drl_actuator init sucess in position: " << _init_pos[0] <<" [ m ] "<<_init_pos[1] <<" [ m ] "<< TAIL <<endl;
}

void drl_actuator::reset(Eigen::Vector3d _init_pos, double _init_yaw)
{
    // 初始化 agent_state
    agent_state.pos = _init_pos;
    agent_state.vel << 0.0,0.0,0.0;
    agent_state.euler << 0.0,0.0,_init_yaw;
    agent_state.quat2 = quaternion_from_rpy(agent_state.euler);
    agent_state.quat.x = agent_state.quat2.x();
    agent_state.quat.y = agent_state.quat2.y();
    agent_state.quat.z = agent_state.quat2.z();
    agent_state.quat.w = agent_state.quat2.w();

    cmd_id = 0;
    discreated_action.ID = 0;
    discreated_action.CMD = prometheus_drl::move_cmd::HOLD;
}

void drl_actuator::move_cmd_cb(const prometheus_drl::move_cmd::ConstPtr& msg)
{
    discreated_action = *msg;

    if(discreated_action.ID <= cmd_id)
    {
        cout << RED << node_name << "---> wrong cmd id."<< TAIL << endl;
        return;
    }
    cmd_id = discreated_action.ID;
    get_move_cmd = true;

    if(discreated_action.CMD == prometheus_drl::move_cmd::HOLD)
    {

    }else if(discreated_action.CMD == prometheus_drl::move_cmd::FORWARD)
    {
        agent_state.pos[0] = agent_state.pos[0] + block_size;
    }else if(discreated_action.CMD == prometheus_drl::move_cmd::BACK)
    {
        agent_state.pos[0] = agent_state.pos[0] - block_size;
    }else if(discreated_action.CMD == prometheus_drl::move_cmd::LEFT)
    {
        agent_state.pos[1] = agent_state.pos[1] + block_size;
    }else if(discreated_action.CMD == prometheus_drl::move_cmd::RIGHT)
    {
        agent_state.pos[1] = agent_state.pos[1] - block_size;
    }else
    {
        cout << RED << node_name << "---> wrong move cmd."<< TAIL << endl;
    }
}

void drl_actuator::vel_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    continued_action = *msg;
    if(actuator_model == 0)
    {
        // 默认vel_cmd_cb的回调频率是10Hz
        // 全向小车
        agent_state.euler << 0.0,0.0,0.0;
        agent_state.quat2 = quaternion_from_rpy(agent_state.euler);
        agent_state.quat.x = agent_state.quat2.x();
        agent_state.quat.y = agent_state.quat2.y();
        agent_state.quat.z = agent_state.quat2.z();
        agent_state.quat.w = agent_state.quat2.w();

        agent_state.vel[0] = msg->linear.x;
        agent_state.vel[1] = msg->linear.y;
        agent_state.vel[2] = 0.0;

        agent_state.pos = agent_state.pos + agent_state.vel * dt;
    }else if(actuator_model == 1)
    {
        // 默认vel_cmd_cb的回调频率是10Hz
        // 差速模型小车
        agent_state.euler[0] = 0.0;
        agent_state.euler[1] = 0.0;
        agent_state.euler[2] = agent_state.euler[2] + msg->angular.z * dt;

        agent_state.quat2 = quaternion_from_rpy(agent_state.euler);
        agent_state.quat.x = agent_state.quat2.x();
        agent_state.quat.y = agent_state.quat2.y();
        agent_state.quat.z = agent_state.quat2.z();
        agent_state.quat.w = agent_state.quat2.w();

        agent_state.vel_body[0] = msg->linear.x;
        agent_state.vel_body[1] = 0.0;
        agent_state.vel_body[2] = 0.0;

        agent_state.vel[0] = agent_state.vel_body[0] * cos(agent_state.euler[2]);
        agent_state.vel[1] = agent_state.vel_body[0] * sin(agent_state.euler[2]);
        agent_state.vel[2] = 0.0;

        agent_state.pos = agent_state.pos + agent_state.vel * dt;
    }

}

void drl_actuator::fake_odom_pub_cb(const ros::TimerEvent &e)
{
    // 发布fake odom
    fake_odom.header.stamp = ros::Time::now();
    fake_odom.header.frame_id = "world";
    fake_odom.child_frame_id = "base_link";
    fake_odom.pose.pose.position.x = agent_state.pos[0];
    fake_odom.pose.pose.position.y = agent_state.pos[1];
    fake_odom.pose.pose.position.z = agent_state.pos[2];
    fake_odom.pose.pose.orientation = agent_state.quat;
    fake_odom.twist.twist.linear.x = agent_state.vel[0];
    fake_odom.twist.twist.linear.y = agent_state.vel[1];
    fake_odom.twist.twist.linear.z = agent_state.vel[2];
    fake_odom_pub.publish(fake_odom);

    // 发布mesh
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = "world";
    meshROS.header.stamp = ros::Time::now();
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = agent_state.pos[0];
    meshROS.pose.position.y = agent_state.pos[1];
    meshROS.pose.position.z = agent_state.pos[2];
    meshROS.pose.orientation = agent_state.quat;

    if(agent_prefix == "/uav")
    {
        meshROS.scale.x = 2.0;
        meshROS.scale.y = 2.0;
        meshROS.scale.z = 2.0;
        meshROS.color.a = 1.0;
        meshROS.color.r = 0.0;
        meshROS.color.g = 0.0;
        meshROS.color.b = 1.0;
        meshROS.mesh_resource = std::string("package://prometheus_drl/meshes/hummingbird.mesh");
    }else if(agent_prefix == "/ugv")
    {
        meshROS.scale.x = 0.8/4.5;
        meshROS.scale.y = 0.8/4.5;
        meshROS.scale.z = 0.8/4.5;
        meshROS.color.a = 1.0;
        meshROS.color.r = 0.0;
        meshROS.color.g = 0.0;
        meshROS.color.b = 0.5;
        meshROS.mesh_resource = std::string("package://prometheus_drl/meshes/car.dae");
    }

    mesh_pub.publish(meshROS); 


    // 发布TF用于RVIZ显示
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";  //相对于世界坐标系
    tfs.header.stamp = ros::Time::now();  //时间戳
    
    //  |----坐标系 ID
    tfs.child_frame_id =  agent_prefix + std::to_string(agent_id) + "/lidar_link";  //子坐标系，无人机的坐标系

    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = agent_state.pos[0];
    tfs.transform.translation.y = agent_state.pos[1];
    tfs.transform.translation.z = agent_state.pos[2];
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
    gazebo_model_state.pose.position.x = agent_state.pos[0];
    gazebo_model_state.pose.position.y = agent_state.pos[1];
    gazebo_model_state.pose.position.z = agent_state.pos[2];
    gazebo_model_state.pose.orientation = agent_state.quat;
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

    if(action_mode == 0)
    {
        if(discreated_action.CMD == prometheus_drl::move_cmd::HOLD)
        {
            cout << GREEN  << "Action : [" << discreated_action.ID << "] " << "[  HOLD   ]" << TAIL <<endl;
        }else if(discreated_action.CMD == prometheus_drl::move_cmd::FORWARD)
        {
            cout << GREEN  << "Action : [" << discreated_action.ID << "] " << "[ FORWARD ]" << TAIL <<endl;
        }else if(discreated_action.CMD == prometheus_drl::move_cmd::BACK)
        {
            cout << GREEN  << "Action : [" << discreated_action.ID << "] " << "[  BACK   ]" << TAIL <<endl;
        }else if(discreated_action.CMD == prometheus_drl::move_cmd::LEFT)
        {
            cout << GREEN  << "Action : [" << discreated_action.ID << "] " << "[  LEFT   ]" << TAIL <<endl;
        }else if(discreated_action.CMD == prometheus_drl::move_cmd::RIGHT)
        {
            cout << GREEN  << "Action : [" << discreated_action.ID << "] " << "[  RIGHT  ]" << TAIL <<endl;
        }
    }else if(action_mode == 1)
    {

    }



}

Eigen::Vector3d drl_actuator::get_agent_pos()
{
    return agent_state.pos;
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